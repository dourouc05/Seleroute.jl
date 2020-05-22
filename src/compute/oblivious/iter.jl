# Useful data structures for all iterative algorithms.
struct ConsideredTrafficMatrix
    subproblem_objective::Float64
    subproblem_edge::Edge
    matrix::Dict{Edge, Float64}
end

function ConsideredTrafficMatrix(objective::Float64, edge::Edge, matrix)
    matrix_as_dict = Dict(k.I[1] => matrix[k] for k in keys(matrix))
    return ConsideredTrafficMatrix(objective, edge, matrix_as_dict)
end

# Interface for a formulation and default implementations (only providing error messages).

oblivious_subproblem_model(m::Model, rd::RoutingData, e_bar::Edge, current_routing::Routing, ::Val...) =
    error("Not implemented: $(rd.sub_model_type)")

add_traffic_matrix(rm::RoutingModel, matrix::ConsideredTrafficMatrix, x::Val...) =
    add_traffic_matrix(rm, matrix.matrix, x...) # Just for unboxing.
add_traffic_matrix(rm::RoutingModel, matrix::Dict{Edge, Float64}, ::Val...) =
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
solve_subproblem(rd::RoutingData, ::RoutingModel, rm::RoutingModel, e_bar::Edge, ::Val...) =
    error("Model type not yet implemented: $(rd.model_type)")

# Default implementation of the master problem and subproblem. It works without trouble if there is no column generation.
function solve_master_problem(rd::RoutingData, rm::RoutingModel, ::Load, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::ObliviousUncertainty, ::CuttingPlane, ::UncertainDemand)
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
        routing_v = value.(s_rm.routing)
        demand = Dict(d => sum(routing_v[d, p_id] for p_id in rd.demand_to_path_ids[d]) for d in demands(rd))
        candidate_matrix = ConsideredTrafficMatrix(objective_value(s_rm.model), e_bar, demand)
    end

    return s_result, 0, candidate_matrix
end

function add_traffic_matrix(rm::RoutingModel, matrix::Dict{Edge, Float64}, ::FormulationType)
    # Compute the congestion for this traffic matrix if asked for.
    opt_d = if rm.data.model_exact_opt_d
        compute_max_load(rm.data, matrix) # Exact value for this traffic matrix (1.0, with numerical errors).
    else
        1.0 # Theoretical value.
    end

    # Add a few capacity constraints.
    @assert ! (matrix in keys(rm.constraints_matrices))
    rm.constraints_matrices[matrix] = Dict{Edge, Any}()
    nb_added_cuts = 0 # Cannot update nb_added_cuts all at once, due to the condition within the loop.
    for e in edges(rm.data)
        flow = total_flow_in_edge(rm, e, matrix)
        if ! iszero(flow)
            rm.constraints_matrices[matrix][e] = @constraint(rm.model, flow <= rm.mu * get_prop(rm.data.g, e, :capacity) * opt_d)
            nb_added_cuts += 1
        end
    end

    return nb_added_cuts
end

function oblivious_subproblem_model(m::Model, rd::RoutingData, e_bar::Edge, current_routing::Routing, type::FormulationType)
    # This function may use another formulation than the master. As the communication between the two is limited to
    # traffic matrices, this is no problem.
    # TODO: Think about heuristics to solve the subproblem.

    @variable(m, demand[d in demands(rd)] >= 0)
    rm = basic_routing_model_unscaled(m, rd, demand, type)

    # The demand matrix must respect the capacity constraints and the existing routing.
    # In other words, normalise the matrix so that its optimum congestion OPT(D) is 1.
    # Not a unitary flow!
    capacity_constraints(rm)

    # Objective function: maximise the congestion over the edge \bar{e}.
    obj = total_flow_in_edge(current_routing, e_bar, demand)
    obj /= get_prop(rd.g, e_bar, :capacity)
    @objective(m, Max, obj)

    return rm
end

# Main loop for all iterative implementations of oblivious routing for uncertain demand.
# Also works with column generation.

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, ::FormulationType, ::Val, ::CuttingPlane, ::ObliviousUncertainty, ::UncertainDemand)
    start = time_ns()

    # Create the master problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)
    @variable(m, mu >= 0)
    rm.mu = mu
    @objective(m, Min, mu)

    objectives = Float64[]
    routings = Routing[]
    matrices = Dict{Edge, Float64}[]
    matrices_set = Set{Dict{Edge, Float64}}()
    it = 1
    total_matrices = 0
    total_cuts = 0
    total_new_paths = 0

    while true
        # Solve the current master problem, possibly with column generation.
        result, current_routing, n_new_paths = solve_master_problem(rd, rm, rd.model_type)
        total_new_paths += n_new_paths

        if rd.export_lps
            write_LP("$(rd.output_folder)/lp_master_$(it).lp", m)
        end

        # If the master problem could not be solved, save it and quit.
        if result != MOI.OPTIMAL
            if rd.export_lps_on_error
                write_LP("$(rd.output_folder)/error_master_$(it).lp", m)
            end

            rd.logmessage("Current master problem could not be solved! Error code: $(result)")
            error("Current master problem could not be solved! Error code: $(result)")
        end

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
            s_m = Model(rd.solver)
            set_silent(s_m)
            s_rm = oblivious_subproblem_model(s_m, rd, e_bar, current_routing)

            s_result, n_sub_new_paths, candidate_matrix = solve_subproblem(rd, rm, s_rm, e_bar, obj, formulation, cg, algo, unc, uncparams)
            n_new_paths_this_iter += n_sub_new_paths
            total_new_paths += n_sub_new_paths

            if rd.export_lps
                write_LP("$(rd.output_folder)/lp_subproblem_$(it)_edge_$(e_bar.src)_to_$(e_bar.dst).lp", s_m)
            end

            # If it is not feasible, exit right now.
            if s_result != MOI.OPTIMAL
                if rd.export_lps_on_error
                    write_LP("$(rd.output_folder)/error_subproblem_$(it)_edge_$(e_bar.src)_to_$(e_bar.dst).lp", s_m)
                end

                rd.logmessage("Subproblem could not be solved! Error code: $(s_result)")
                error("Subproblem could not be solved! Error code: $(s_result)")
            end

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
        for matrix in interesting_matrices
            if matrix.matrix in matrices_set
                continue
            end
            push!(matrices_set, matrix.matrix)
            push!(matrices, matrix.matrix)

            # Add the matrix to the model.
            nb_added_cuts += add_traffic_matrix(rm, matrix)
        end

        total_matrices += length(interesting_matrices)
        total_cuts += nb_added_cuts

        rd.logmessage("Considered $(length(interesting_matrices)) matri$(ifelse(length(interesting_matrices) == 1, "x", "ces")).")
        if nb_added_cuts == 0
            rd.logmessage("Did not add any new constraint. Done!")
            break
        end
        rd.logmessage("Added $(nb_added_cuts) more constraint$(ifelse(nb_added_cuts == 1, "", "s")), going on!")
        rd.logmessage("Added $(n_new_paths) more path$(ifelse(n_new_paths == 1, "", "s")) for the master problem!")
        rd.logmessage("Added $(n_new_paths_this_iter) more path$(ifelse(n_new_paths_this_iter == 1, "", "s")) for the subproblems!")

        # If needed, plot the results. Don't plot for the last iteration,
        # as this is automatically performed (the whole solution is always
        # plotted after the main loop).
        if plot_each_iteration
            rd.logmessage("== DBG == Starting to plot the results...")
            # plot(rd, current_routing, basename="$(rd.output_folder)/graph_it$(it)")
        end

        # Prepare for the next iteration.
        it += 1
    end

    stop = time_ns()

    # Do the final round of plotting.
    rd.logmessage("== DBG == Starting to plot the results...")
    # plot(rd, routings[end], basename="$(rd.output_folder)/graph_final")
    # rd.logmessage("== DBG == Generating the textual summary...")
    # summary(rd, routings[1], routings[end], filename="$(rd.output_folder)/psummary.txt")
    rd.logmessage("== DBG == Exporting the solution...")
    # export_routing(rd, routings[end], filename="$(rd.output_folder)/oblivious_routing.txt")

    exported = time_ns()

    return RoutingSolution(rd, it, total_matrices, total_cuts, total_new_paths,
                           rd.time_precompute_ms, (stop - start) / 1_000_000., (exported - stop) / 1_000_000.,
                           objectives, matrices, routings, rm)
end
