# https://www.ibm.com/support/knowledgecenter/SSSA5P_12.10.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/EpOpt.html
const CPLEX_REDUCED_COST_TOLERANCE = 1.0e-6

function _check_weight_matrix(weight_matrix)
    # Return true if there may be a new path in this weight matrix, false if it is hopeless.

    # If all weights are zero, impossible to get a negative reduced cost.
    # Very efficient test, unlike the next one.
    if nnz(weight_matrix) == 0
        return false
    end

    # If all weights are negative, there is a negative-cost cycle.
    return ! all(weight_matrix .<= CPLEX_REDUCED_COST_TOLERANCE)
end

function _build_path(state::LightGraphs.AbstractPathState, demand::Edge)
    reverse_nodes = [dst(demand)]
    current_node = dst(demand)
    while current_node != src(demand)
        current_node = state.parents[current_node]
        push!(reverse_nodes, current_node)
    end
    return [Edge(reverse_nodes[i], reverse_nodes[i - 1]) for i in length(reverse_nodes):-1:2]
end

function _solve_pricing_problem_master(rd::RoutingData, dual_values_matrices, dual_value_convexity, matrices::Vector{Dict{Edge, Float64}})
    paths = Dict{Edge, Vector{Edge}}()

    for demand in demands(rd)
        # Determine the weight matrix to use for the computations:
        #     for e: \sum_{D\in\Delta} dual_{e, d_k} d_k
        rd.logmessage("Pricing for $demand")
        weight_matrix = spzeros(Float64, n_edges(rd), n_edges(rd))
        for e in edges(rd)
            for matrix in matrices
                if e in keys(dual_values_matrices[matrix])
                    # Ignore positive dual values, they are mostly numerical errors.
                    if dual_values_matrices[matrix][e] >= - CPLEX_REDUCED_COST_TOLERANCE
                        # rd.logmessage("[$matrix][$e] $(dual_values_matrices[matrix][e])")
                        continue
                    end

                    weight_matrix[src(e), dst(e)] -= matrix[demand] * dual_values_matrices[matrix][e]
                end
            end
        end

        if ! _check_weight_matrix(weight_matrix)
            continue
        end

        # Compute the shortest path for this demand.
        state = desopo_pape_shortest_paths(rd.g, src(demand), weight_matrix)

        # Is there a solution? Has this path a negative reduced cost?
        if all(state.parents .== 0)
            continue
        end
        if state.dists[dst(demand)] - dual_value_convexity[demand] >= - CPLEX_REDUCED_COST_TOLERANCE
            continue
        end
        rd.logmessage(state.dists[dst(demand)] - dual_value_convexity[demand])

        # Build the path.
        path = _build_path(state, demand)
        if path in rd.paths_edges
            rd.logmessage("New path $(path) is already in the formulation!")
        else
            paths[demand] = path
        end
    end

    return paths
end

function _solve_pricing_problem_subproblem(rd::RoutingData, dual_values_capacity)
    paths = Dict{Edge, Vector{Edge}}()

    # Determine the weight matrix to use for the computations.
    weight_matrix = spzeros(Float64, n_edges(rd), n_edges(rd))
    for e in keys(dual_values_capacity)
        # Ignore negative dual values, they are mostly numerical errors.
        if dual_values_capacity[e] <= CPLEX_REDUCED_COST_TOLERANCE
            # rd.logmessage("[$e] Ignoring dual value: $(dual_values_capacity[e])")
            continue
        end

        weight_matrix[src(e), dst(e)] -= dual_values_capacity[e]
    end

    if ! _check_weight_matrix(weight_matrix)
        return paths
    end

    # Perform the actual pricing.
    for demand in demands(rd)
        # Compute the shortest path for this demand.
        state = desopo_pape_shortest_paths(rd.g, src(demand), weight_matrix)

        # Is there a solution? Has this path a negative reduced cost?
        if all(state.parents .== 0)
            continue
        end
        # TODO: state.dists[dst(demand)] - dual_value_convexity[demand] to avoid cycling!?
        if state.dists[dst(demand)] >= - CPLEX_REDUCED_COST_TOLERANCE
            continue
        end
        rd.logmessage(state.dists[dst(demand)])

        # Build the path.
        path = _build_path(state, demand)
        if path in rd.paths_edges
            rd.logmessage("New path $(path) is already in the formulation!")
        else
            paths[demand] = path
        end
    end

    return paths
end

function _add_path_to_data(rd::RoutingData, demand::Edge, path::Vector{Edge})
    # Update RoutingData with the new path.
    push!(rd.paths_edges, path)
    path_id = length(rd.paths_edges)
    rd.path_id_to_demand[path_id] = demand

    # All demands already have a path, by assumption. Otherwise, the master problem should not be feasible.
    # Hence, computing the new path ID is easy.
    push!(rd.demand_to_path_ids[demand], path_id)

    return path_id
end

function _add_path_to_master_formulation(rd::RoutingData, rm::RoutingModel, demand::Edge, path_id::Int)
    # Update RoutingModel. Create a new variable for the new path, add it in the corresponding constraints.

    # New variable.
    rm.routing[demand, path_id] = @variable(rm.model, lower_bound=0, upper_bound=1)
    set_name(rm.routing[demand, path_id], "routing[$demand, $path_id]")

    # Convexity constraint.
    set_normalized_coefficient(rm.constraints_convexity[demand], rm.routing[demand, path_id], 1)

    # Traffic constraints.
    for matrix in keys(rm.constraints_matrices)
        if iszero(matrix[demand])
            continue
        end

        for e in keys(rm.constraints_matrices[matrix])
            if e in rd.paths_edges[path_id]
                set_normalized_coefficient(rm.constraints_matrices[matrix][e], rm.routing[demand, path_id], matrix[demand])
            end
        end
    end
end

function _add_path_to_subproblem_formulation(rd::RoutingData, s_rm::RoutingModel, demand::Edge, path_id::Int)
    # Update RoutingModel. Create a new variable for the new path, add it in the corresponding constraints.

    # New variable.
    s_rm.routing[demand, path_id] = @variable(s_rm.model, lower_bound=0, upper_bound=1)
    set_name(s_rm.routing[demand, path_id], "routing[$demand, $path_id]")

    # Capacity constraints.
    for e in edges(rd)
        if e in rd.paths_edges[path_id]
            set_normalized_coefficient(s_rm.constraints_capacity[e], s_rm.routing[demand, path_id], 1)
        end
    end
end

function solve_master_problem(rd::RoutingData, rm::RoutingModel, ::Val{Load}, ::Val{MinimumMaximum}, formulation::Val, ::Val{true}, ::Val{ObliviousUncertainty}, ::Val{CuttingPlane}, ::Val{UncertainDemand})
    n_new_paths = 0
    @assert rm.mu !== nothing

    result = nothing
    current_routing = nothing

    while true
        # Solve the current master problem.
        m = rm.model
        optimize!(m)
        result = termination_status(m)
        current_routing = Routing(rd, value.(rm.routing))

        # Check if there are still columns to add.
        dual_values_matrices = Dict(tm => Dict(e => dual(constraint) for (e, constraint) in d) for (tm, d) in rm.constraints_matrices)
        dual_value_convexity = dual.(rm.constraints_convexity)
        matrices = collect(keys(rm.constraints_matrices))

        new_paths = _solve_pricing_problem_master(rd, dual_values_matrices, dual_value_convexity, matrices)
        n_new_paths += length(new_paths)

        if length(new_paths) == 0
            break
        end

        # Add the new paths to the formulation.
        for (d, path) in new_paths
            path_id = _add_path_to_data(rd, d, path)
            _add_path_to_master_formulation(rd, rm, d, path_id)
        end
    end

    return result, current_routing, n_new_paths
end

function solve_subproblem(rd::RoutingData, rm::RoutingModel, s_rm::RoutingModel, e_bar::Edge, ::Val{Load}, ::Val{MinimumMaximum}, formulation::Val, ::Val{true}, ::Val{ObliviousUncertainty}, ::Val{CuttingPlane}, ::Val{UncertainDemand})
    n_new_paths = 0
    result = nothing

    while true
        # Solve the current subproblem.
        result = termination_status(s_rm.model)

        # Check if there are still columns to add.
        dual_values_capacity = Dict(e => dual(constraint) for (e, constraint) in s_rm.constraints_capacity)

        rd.logmessage("Subpricing for edge $e_bar")
        new_paths = _solve_pricing_problem_subproblem(rd, dual_values_capacity)
        n_new_paths += length(new_paths)

        if length(new_paths) == 0
            break
        end

        # Add the new paths to the formulation.
        for (d, path) in new_paths
            path_id = _add_path_to_data(rd, d, path)
            _add_path_to_master_formulation(rd, rm, d, path_id)
            _add_path_to_subproblem_formulation(rd, s_rm, d, path_id)
        end
    end

    routing_v = value.(s_rm.routing)
    demand = Dict(d => sum(routing_v[d, p_id] for p_id in rd.demand_to_path_ids[d]) for d in demands(rd))
    candidate_matrix = ConsideredTrafficMatrix(objective_value(s_rm.model), e_bar, demand)

    return result, n_new_paths, candidate_matrix
end
