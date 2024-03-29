# TODO: replace this with a solver-agnostic tolerance?
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

function _build_path(state::Graphs.AbstractPathState, demand::Edge)
    # TODO: use Graphs.enumerate_paths? It returns lists of vertices and not edges...
    reverse_nodes = [dst(demand)]
    current_node = dst(demand)
    while current_node != src(demand)
        current_node = state.parents[current_node]
        push!(reverse_nodes, current_node)
    end
    return [Edge(reverse_nodes[i], reverse_nodes[i - 1]) for i in length(reverse_nodes):-1:2]
end

function _oblivious_iterative_solve_pricing_problem_master(rd::RoutingData,
                                                           dual_values_matrices,
                                                           dual_value_convexity,
                                                           matrices::Vector{Dict{Edge{Int}, Float64}},
                                                           timeout::Period)
    # Return paths to add to the formulation (as a dictionary: demand and the
    # list of paths for that demand) and a status (MOI.OPTIMAL if the pricing
    # was performed without problem or MOI.TIME_LIMIT if the timeout was
    # reached). 
    paths = Dict{Edge{Int}, Vector{Edge}}()
    result = MOI.OPTIMAL
    start = time_ns()

    for demand in demands(rd)
        # Enforce the timeout.
        currently_elapsed_time = Nanosecond(time_ns() - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end

        # Determine the weight matrix to use for the computations:
        #     for e: \sum_{D\in\Delta} dual_{e, d_k} d_k
        rd.logmessage("Pricing for $demand")
        weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
        for e in edges(rd)
            for matrix in matrices
                if e in keys(dual_values_matrices[matrix])
                    # Ignore positive dual values, they are mostly numerical errors.
                    if dual_values_matrices[matrix][e] >= - CPLEX_REDUCED_COST_TOLERANCE
                        continue
                    end

                    weight_matrix[src(e), dst(e)] -= matrix[demand] * dual_values_matrices[matrix][e]
                end
            end
        end

        # Check that at least some weights are nonzero.
        if ! _check_weight_matrix(weight_matrix)
            continue
        end

        # Compute the shortest path for this demand.
        state = dijkstra_shortest_paths(rd.g, src(demand), weight_matrix)

        # Is there a solution? Has this path a negative reduced cost?
        if all(state.parents .== 0)
            continue
        end
        reduced_cost = state.dists[dst(demand)] - dual_value_convexity[demand]
        if reduced_cost >= - CPLEX_REDUCED_COST_TOLERANCE
            continue
        end
        rd.logmessage(reduced_cost)

        # Build the path.
        path = _build_path(state, demand)
        if path in rd.paths_edges
            rd.logmessage("New path $(path) is already in the formulation!")
        else
            paths[demand] = path
        end
    end

    return paths, result
end

function _oblivious_iterative_solve_pricing_problem_subproblem(
        rd::RoutingData, dual_values_capacity, timeout::Period)
    # Return paths to add to the formulation (as a dictionary: demand and the
    # list of paths for that demand) and a status (MOI.OPTIMAL if the pricing
    # was performed without problem or MOI.TIME_LIMIT if the timeout was
    # reached). 
    paths = Dict{Edge{Int}, Vector{Edge}}()
    result = MOI.OPTIMAL
    start = time_ns()

    # Determine the weight matrix to use for the computations.
    weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
    for e in keys(dual_values_capacity)
        # Ignore negative dual values, they are mostly numerical errors.
        if dual_values_capacity[e] <= CPLEX_REDUCED_COST_TOLERANCE
            # rd.logmessage("[$e] Ignoring dual value: $(dual_values_capacity[e])")
            continue
        end

        weight_matrix[src(e), dst(e)] -= dual_values_capacity[e]
    end

    if ! _check_weight_matrix(weight_matrix)
        return paths, result
    end

    # Perform the actual pricing.
    for demand in demands(rd)
        # Enforce the timeout.
        currently_elapsed_time = Nanosecond(time_ns() - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end

        # Compute the shortest path for this demand.
        state = dijkstra_shortest_paths(rd.g, src(demand), weight_matrix)

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

    return paths, result
end

function _add_path_to_master_formulation(rd::RoutingData, rm::RoutingModel, demand::Edge, path_id::Int)
    # Update RoutingModel. Create a new variable for the new path, add it in the corresponding constraints.

    # New variable and convexity constraint.
    add_routing_var!(rm, demand, path_id)

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

    add_routing_var!(s_rm, demand, path_id)

    # New variable.
    s_rm.routing[demand, path_id] = @variable(s_rm.model, lower_bound=0, upper_bound=1)
    set_name(s_rm.routing[demand, path_id], "routing[$demand, $path_id]")

    # Capacity constraints.
    for e in edges(rd)
        if e in rd.paths_edges[path_id]
            set_normalized_coefficient(
                s_rm.constraints_capacity[e], s_rm.routing[demand, path_id], 1)
        end
    end
end

function solve_master_problem(rd::RoutingData, rm::RoutingModel, ::Load,
                              ::MinimumMaximum, ::PathFormulation, ::Val{true},
                              ::CuttingPlane, ::ObliviousUncertainty,
                              ::UncertainDemand, timeout::Period)
    @assert rm.mu !== nothing
    start = time_ns()
    n_new_paths = 0

    result = nothing
    current_routing = nothing
    current_routing_nb_paths = 0

    while true
        start_iter = time_ns()

        # Enfore the timeout (the argument has precedence over the value in
        # RoutingData, because it depends on the time elapsed in previous
        # iterations).
        currently_elapsed_time = Nanosecond(start_iter - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end
    
        if remaining_timeout.value > 0
            set_time_limit_sec(rm.model, floor(remaining_timeout, Second).value)
        end

        # Solve the current master problem.
        optimize!(rm.model)
        result = termination_status(rm.model)
        current_routing = Routing(rd, value.(rm.routing)) # TODO: remember all
        # generated routings. For now, there is only one routing memorised for
        # each outer-loop iteration (when addind new matrices).
        current_routing_nb_paths = count(value.(rm.routing) .>= CPLEX_REDUCED_COST_TOLERANCE)

        # Check if there are still columns to add. Also enforce the time out.
        dual_values_matrices = Dict(tm => Dict(e => dual(constraint) for (e, constraint) in d) for (tm, d) in rm.constraints_matrices)
        dual_value_convexity = dual.(rm.constraints_convexity)
        matrices = collect(keys(rm.constraints_matrices))

        currently_elapsed_time = Nanosecond(time_ns() - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end

        new_paths, pricing_result =
            _oblivious_iterative_solve_pricing_problem_master(
                rd, dual_values_matrices, dual_value_convexity, matrices,
                remaining_timeout)
        if pricing_result != MOI.OPTIMAL
            result = pricing_result
            break
        end
        n_new_paths += length(new_paths)

        if length(new_paths) == 0
            break
        end

        # Add the new paths to the formulation.
        for (d, path) in new_paths
            path_id = add_path!(rd, d, path)
            _add_path_to_master_formulation(rd, rm, d, path_id)
        end

        # TODO: propose to output problems at each iteration (_export_lp_if_allowed).
    end

    return result, current_routing, n_new_paths, current_routing_nb_paths
end

function solve_subproblem(rd::RoutingData, rm::RoutingModel,
                          s_rm::RoutingModel, e_bar::Edge, ::Load,
                          ::MinimumMaximum, ::FormulationType, ::Val{true},
                          ::CuttingPlane, ::ObliviousUncertainty,
                          ::UncertainDemand, timeout::Period)    
    # Solve the subproblem.
    n_new_paths = 0
    result = nothing
    start = time_ns()

    while true
        start_iter = time_ns()
        # TODO: store the timings for each iteration?

        # Enfore the timeout (the argument has precedence over the value in
        # RoutingData, because it depends on the time elapsed in previous
        # iterations).
        currently_elapsed_time = Nanosecond(start_iter - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end
    
        if remaining_timeout.value > 0
            set_time_limit_sec(rm.model, floor(remaining_timeout, Second).value)
        end

        # Solve the current subproblem.
        optimize!(s_rm.model)
        result = termination_status(s_rm.model)

        # Check if there are still columns to add.
        dual_values_capacity = Dict(e => dual(constraint) for (e, constraint) in s_rm.constraints_capacity)

        currently_elapsed_time = Nanosecond(time_ns() - start)
        remaining_timeout = convert(Nanosecond, (if timeout.value > 0
            timeout
        elseif rd.timeout.value > 0
            remaining_timeout = rd.timeout
        else
            Nanosecond(0)
        end) - currently_elapsed_time)
        
        if (rd.timeout.value > 0 || timeout.value > 0) && currently_elapsed_time >= remaining_timeout
            result = MOI.TIME_LIMIT
            break
        end

        rd.logmessage("Subpricing for edge $e_bar")
        new_paths, pricing_result =
            _oblivious_iterative_solve_pricing_problem_subproblem(
                rd, dual_values_capacity, remaining_timeout)
        if pricing_result != MOI.OPTIMAL
            result = pricing_result
            break
        end
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

        # TODO: propose to output problems at each iteration (_export_lp_if_allowed).
    end

    routing_v = value.(s_rm.routing)
    demand = Dict(d => sum(routing_v[d, p_id] for p_id in rd.demand_to_path_ids[d]) for d in demands(rd))
    candidate_matrix = ConsideredTrafficMatrix(objective_value(s_rm.model), e_bar, demand)

    return result, n_new_paths, candidate_matrix
end
