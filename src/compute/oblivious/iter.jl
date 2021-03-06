# Based on Optimal Oblivious Routing in Polynomial Time
# http://ttic.uchicago.edu/~harry/pdf/optimal_oblivious_journal.pdf
# Oblivious ratio: O(log^2 n log log n), n number of nodes
# http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.4.3883&rep=rep1&type=pdf: theorem 1, p. 9

# Useful data structures for all oblivious iterative algorithms.
# TODO: also makes sense for robust, move it upwards?
struct ConsideredTrafficMatrix{T}
    subproblem_objective::Float64
    subproblem_edge::Edge
    matrix::Dict{Edge{T}, Float64}
end

function ConsideredTrafficMatrix(objective::Float64, edge::Edge, matrix::JuMP.Containers.DenseAxisArray)
    # Complete type for matrix:
    # {Float64,1,Tuple{LightGraphs.SimpleGraphs.SimpleEdgeIter{SimpleDiGraph{Int64}}},Tuple{Dict{LightGraphs.SimpleGraphs.SimpleEdge{Int64},Int64}}}
    matrix_as_dict = Dict(k => matrix[k] for k in keys(matrix))
    return ConsideredTrafficMatrix(objective, edge, matrix_as_dict)
end

function ConsideredTrafficMatrix(objective::Float64, edge::Edge, matrix)
    matrix_as_dict = Dict(k.I[1] => matrix[k] for k in keys(matrix))
    return ConsideredTrafficMatrix(objective, edge, matrix_as_dict)
end


# Interface for a formulation and default implementations (only providing error messages).

oblivious_subproblem_model(m::Model, rd::RoutingData, e_bar::Edge, current_routing::Routing, ::Any...) =
    error("Not implemented: $(rd.sub_model_type)")

add_traffic_matrix(rm::RoutingModel, matrix::ConsideredTrafficMatrix) =
    add_traffic_matrix(rm, matrix.matrix) # Unbox ConsideredTrafficMatrix.
add_traffic_matrix(rm::RoutingModel, matrix::Dict{Edge{Int}, Float64}) =
    add_traffic_matrix(rm, matrix, rm.data.model_type.type) # Guess the formulation type.
add_traffic_matrix(rm::RoutingModel, matrix::Dict{Edge{Int}, Float64}, ::Any) =
    error("Not implemented: $(rm.data.model_type)")

"""
Return values: `result`, `current_routing`, `n_new_paths`.
"""
solve_master_problem(rd::RoutingData, rm::RoutingModel, ::Any...) =
    error("Model type not yet implemented: $(rd.model_type)")

solve_master_problem(rd::RoutingData, rm::RoutingModel, mt::ModelType) =
    solve_master_problem(rd, rm, mt.edge_obj, mt.agg_obj, mt.type, Val(mt.cg), mt.algo, mt.unc, mt.uncparams)

"""
Return values: `result`, `n_new_paths`, `candidate_matrix`.
"""
solve_subproblem(rd::RoutingData, ::RoutingModel, rm::RoutingModel, e_bar::Edge, ::Any...) =
    error("Model type not yet implemented: $(rd.model_type)")

# Default implementation of the master problem and subproblem. It works without trouble if there is no column generation.
function solve_master_problem(rd::RoutingData, rm::RoutingModel, ::Load, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::CuttingPlane, ::ObliviousUncertainty, ::UncertainDemand)
    @assert rm.mu !== nothing

    optimize!(rm.model)
    result = termination_status(rm.model)
    current_routing = Routing(rd, value.(rm.routing))

    return result, current_routing, 0
end

function solve_subproblem(rd::RoutingData, ::RoutingModel, s_rm::RoutingModel, e_bar::Edge, ::Load, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::CuttingPlane, ::ObliviousUncertainty, ::UncertainDemand)
    optimize!(s_rm.model)
    s_result = termination_status(s_rm.model)

    if s_rm.demand !== nothing
        candidate_matrix = ConsideredTrafficMatrix(objective_value(s_rm.model), e_bar, value.(s_rm.demand))
    else
        # If there is no demand variable available in the formulation, find it back from the routing.
        routing_v = value.(s_rm.routing)
        demand = Dict(d => sum(routing_v[d, p_id] for p_id in rd.demand_to_path_ids[d]) for d in demands(rd))
        candidate_matrix = ConsideredTrafficMatrix(objective_value(s_rm.model), e_bar, demand)
    end

    return s_result, 0, candidate_matrix
end

function add_traffic_matrix(rm::RoutingModel, matrix::Dict{Edge{Int}, Float64}, ::FormulationType)
    # Compute the congestion for this traffic matrix if asked for.
    opt_d = if rm.data.model_exact_opt_d
        compute_max_load(rm.data, matrix) # Exact value for this traffic matrix (1.0, with numerical errors).
    else
        1.0 # Theoretical value.
    end

    # Add a few capacity constraints.
    @assert ! (matrix in keys(rm.constraints_matrices))
    rm.constraints_matrices[matrix] = Dict{Edge{Int}, Any}()
    nb_added_cuts = 0 # Cannot update nb_added_cuts all at once, due to the condition within the loop.
    for e in edges(rm.data)
        flow = total_flow_in_edge(rm, e, matrix)
        if ! iszero(flow)
            rm.constraints_matrices[matrix][e] = @constraint(rm.model, flow <= rm.mu * capacity(rm, e) * opt_d)
            nb_added_cuts += 1
        end
    end

    return nb_added_cuts
end

function oblivious_subproblem_model(m::Model, rd::RoutingData, e_bar::Edge, current_routing::Routing, type::FormulationType)
    # This function may use another formulation than the master. As the communication between the two is limited to
    # traffic matrices, this is no problem.
    # TODO: Think about heuristics to solve the subproblems faster.

    @variable(m, demand[d in demands(rd)] >= 0)
    rm = basic_routing_model_unscaled(m, rd, demand, type)
    rm.demand = demand

    # The demand matrix must respect the capacity constraints and the existing routing.
    # In other words, normalise the matrix so that its optimum congestion OPT(D) is 1.
    # Not a unitary flow!
    capacity_constraints(rm)

    # Objective function: maximise the congestion over the edge \bar{e}.
    # TODO: objectives.jl?
    obj = total_flow_in_edge(current_routing, e_bar, demand)
    obj /= capacity(rd, e_bar)
    @objective(m, Max, obj)

    return rm
end

# Main loop for all iterative implementations of oblivious routing for uncertain demand.
# Also works with column generation.

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, type::FormulationType, cg::Val, ::CuttingPlane, ::ObliviousUncertainty, ::UncertainDemand)
    # Create the master problem.
    start = time_ns()
    m = _create_model(rd)
    rm = basic_routing_model_unitary(m, rd)
    @variable(m, mu >= 0)
    rm.mu = mu
    @objective(m, Min, mu)

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Initialise data structures to hold all intermediate results.
    objectives = Float64[]
    routings = Routing[]
    matrices = Dict{Int, Vector{Dict{Edge{Int}, Float64}}}()
    matrices_set = Set{Dict{Edge{Int}, Float64}}()
    it = 1
    total_cuts = 0
    total_new_paths = 0
    time_solve_ms = Float64[]

    # Start the oblivious loop.
    while true
        start = time_ns()

        # Solve the current master problem, possibly with column generation.
        result, current_routing, n_new_paths = solve_master_problem(rd, rm, rd.model_type)
        total_new_paths += n_new_paths

        # Export if needed.
        _export_lp_if_allowed(rd, m, "lp_master_$(it)")
        _export_lp_if_failed(rd, result, m, "error_master_$(it)", "Current master problem could not be solved!")

        # Otherwise, go on and save the original routing (the one that does not
        # consider any oblivious demand).
        push!(routings, current_routing)

        # See if there was a change in mu.
        push!(objectives, value(rm.mu))
        rd.logmessage("Current objective: $(objectives[end])")

        # Solve the separation problem (once per edge) and, if needed, add constraints. If no constraints are needed,
        # the process is done.
        rd.logmessage("Solving the subproblem... $(n_edges(rd)) iterations to perform sequentially")
        interesting_matrices = ConsideredTrafficMatrix[]

        n_edges_done = 0
        n_new_paths_this_iter = 0
        for e_bar in edges(rd)
            if n_edges_done % 20 == 0
                rd.logmessage("Solving the subproblem... Edge $(n_edges_done) out of $(n_edges(rd))")
            end

            # Solve the corresponding separation problem.
            s_m = _create_model(rd)
            s_rm = oblivious_subproblem_model(s_m, rd, e_bar, current_routing, type)

            s_result, n_sub_new_paths, candidate_matrix = solve_subproblem(rd, rm, s_rm, e_bar, Load(), MinimumMaximum(), type, cg, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            n_new_paths_this_iter += n_sub_new_paths
            total_new_paths += n_sub_new_paths

            # Export if needed.
            _export_lp_if_allowed(rd, s_m, "lp_subproblem_$(it)_edge_$(e_bar.src)_to_$(e_bar.dst)")
            _export_lp_if_failed(rd, s_result, s_m, "error_subproblem_$(it)_edge_$(e_bar.src)_to_$(e_bar.dst)", "Subproblem could not be solved!")

            # Is the traffic matrix "interesting"? Two meanings, depending on the parameters:
            # - in any case: would this traffic matrix help reach the best oblivious routing? i.e.: is the
            #   corresponding traffic matrix violating the oblivious constraint?
            # - if so:
            #     - only one traffic matrix per iteration: is it better than the current one (if there is any)?
            #     - any number of traffix matrices per iteration: just add it.
            if objective_value(s_m) > objectives[end] # TODO: Add a specific data structure for the subproblem and rather use a function to make the comparison?
                # This matrix could improve the master problem. Should it still be considered?
                add_matrix = rd.model_all_traffic_matrices || length(interesting_matrices) == 0
                replace_top_matrix = ! add_matrix && objective_value(s_m) > interesting_matrices[1].subproblem_objective

                if add_matrix || replace_top_matrix
                    if add_matrix
                        push!(interesting_matrices, candidate_matrix)
                    else
                        interesting_matrices[1] = candidate_matrix
                    end
                end
            end

            n_edges_done += 1
        end

        rd.logmessage("Solving the subproblem... Done! ")
        rd.logmessage("=> Current mu: $(objectives[end])")

        # Is there a constraint to add?
        nb_added_cuts = 0
        matrices[it] = Dict{Edge{Int}, Float64}[]
        for matrix in interesting_matrices
            if matrix.matrix in matrices_set
                continue
            end
            push!(matrices_set, matrix.matrix)
            push!(matrices[it], matrix.matrix)

            # Add the matrix to the model.
            nb_added_cuts += add_traffic_matrix(rm, matrix)
        end

        total_cuts += nb_added_cuts

        rd.logmessage("Considered $(length(interesting_matrices)) matri$(ifelse(length(interesting_matrices) == 1, "x", "ces")).")
        if nb_added_cuts == 0
            rd.logmessage("Did not add any new constraint. Done!")
            break
        end
        rd.logmessage("Added $(nb_added_cuts) more constraint$(ifelse(nb_added_cuts == 1, "", "s")), going on!")
        rd.logmessage("Added $(n_new_paths) more path$(ifelse(n_new_paths == 1, "", "s")) for the master problem!")
        rd.logmessage("Added $(n_new_paths_this_iter) more path$(ifelse(n_new_paths_this_iter == 1, "", "s")) for the subproblems!")

        # Record the time for this iteration.
        push!(time_solve_ms, (time_ns() - start) / 1_000_000.)

        # If needed, plot the results. Don't plot for the last iteration,
        # as this is automatically performed (the whole solution is always
        # plotted after the main loop).
        # if rd.plot_each_iteration # TODO: reintroduce.
        #     rd.logmessage("== DBG == Starting to plot the results...")
        #     # plot(rd, current_routing, basename="$(rd.output_folder)/graph_it$(it)")
        # end

        # Prepare for the next iteration.
        it += 1
    end

    # Do the final round of plotting.
    rd.logmessage("== DBG == Starting to plot the results...")
    # plot(rd, routings[end], basename="$(rd.output_folder)/graph_final")
    # rd.logmessage("== DBG == Generating the textual summary...")
    # summary(rd, routings[1], routings[end], filename="$(rd.output_folder)/psummary.txt")
    rd.logmessage("== DBG == Exporting the solution...")
    # export_routing(rd, routings[end], filename="$(rd.output_folder)/oblivious_routing.txt")

    # exported = time_ns()

    return RoutingSolution(rd,
                           n_cuts=total_cuts,
                           n_columns=total_new_paths,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=time_solve_ms,
                           objectives=objectives,
                           matrices=matrices,
                           routings=routings,
                           master_model=rm)
end
