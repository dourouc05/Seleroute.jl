function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, ::PathFormulation, ::Val{true}, ::DualReformulation, ::ObliviousUncertainty, ::UncertainDemand)
    # For column generation, more dual values are required for the pricing.
    old_tm = rd.model_robust_reformulation_traffic_matrices
    rd.model_robust_reformulation_traffic_matrices = true

    # Create the master model (same as without column generation).
    start = time_ns()
    rm = master_formulation(rd, PathFormulation())
    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Data structures to hold intermediate results.
    result = MOI.OPTIMAL
    n_new_paths = 0
    routings = Routing[]
    current_routing_nb_paths = 0
    objectives = Float64[]
    times_ms = Float64[]
    times_master_ms = Float64[]
    times_sub_ms = Float64[]

    # Main loop.
    while true
        start_iter = time_ns()

        # Enforce the timeout.
        currently_elapsed_time = Nanosecond(start_iter - start)
        if rd.timeout.value > 0 && currently_elapsed_time >= rd.timeout
            result = MOI.TIME_LIMIT
            break
        end

        if rd.timeout.value > 0
            set_time_limit_sec(rm.model, floor(rd.timeout - currently_elapsed_time, Second).value)
        end

        # Solve the current master problem.
        optimize!(rm.model)

        push!(times_master_ms, (time_ns() - start_iter) / 1_000_000.)
        result = termination_status(rm.model)

        if result == MOI.TIME_LIMIT
            break
        end
        
        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, objective_value(rm.model))
        current_routing_nb_paths = count(value.(rm.routing) .>= CPLEX_REDUCED_COST_TOLERANCE)

        # Export if needed. TODO: use iteration number.
        _export_lp_if_allowed(rd, rm.model, "lp_master")
        _export_lp_if_failed(rd, result, rm.model, "error_master", "Subproblem could not be solved!")

        # Check if there are still columns to add.
        dual_value_convexity = dual.(rm.constraints_convexity) # Demand -> dual value
        dual_values_uncer = Dict{Edge{Int}, Dict{Edge{Int}, Float64}}(
            d => Dict(e => NaN for e in edges(rd))
            for d in demands(rd)
        ) # Demand -> edge -> dual value. All NaNs should be rewritten by the next loop.
        for (e, d_cstr) in rm.constraints_uncertainty_set
            for (d, constraint) in d_cstr
                dual_values_uncer[d][e] = dual(constraint)
            end
        end

        start_sub = time_ns()
        new_paths = _oblivious_reformulation_solve_pricing_problem(rd, dual_values_uncer, dual_value_convexity)
        push!(times_sub_ms, (time_ns() - start_sub) / 1_000_000.)
        n_new_paths += length(new_paths)

        # Add the new columns to the formulation.
        for (d, path) in new_paths
            # Add the new columns to the data object and retrieve a path ID.
            path_id = add_path!(rd, d, path)

            # Get your hands dirty in the formulation.
            # - Create a new variable for this path, add it into the
            #   convexity constraint.
            add_routing_var!(rm, demand, path_id)

            for e in edges(rd)
                # - Add it into the uncertainty sets.
                set_normalized_coefficient(rm.constraints_uncertainty_set[e][d], rm.routing[d, path_id], 1)

                # - Create the new constraint.
                @constraint(rm.model, rm.dual_alpha[e, d] + sum(rm.dual_beta[e, e2] for e2 in path) >= 0)
            end
        end

        push!(times_ms, (time_ns() - start) / 1_000_000.)

        if length(new_paths) == 0
            break
        end
    end

    # Restore the original value.
    rd.model_robust_reformulation_traffic_matrices = old_tm

    # Finalise the process.
    matrices = Dict{Edge{Int}, Float64}[]
    if result != MOI.TIME_LIMIT
        # Export if needed.
        result = termination_status(rm.model)
        _export_lp_if_allowed(rd, rm.model, "lp_master")
        _export_lp_if_failed(rd, result, rm.model, "error_master", "Subproblem could not be solved!")

        # Retrieve the traffic matrices, if asked.
        if rd.model_robust_reformulation_traffic_matrices
            matrices = get_traffic_matrices(rm)
        end

        # TODO: Export things like the normal case.
    end

    return RoutingSolution(rd,
                           result=result,
                           n_columns=n_new_paths,
                           n_columns_master=n_new_paths,
                           n_columns_subproblems=0,
                           n_columns_used=current_routing_nb_paths,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_create_subproblems_model_ms=0.0, # No subproblem, only shortest paths.
                           time_solve_ms=times_ms,
                           time_solve_master_model_ms=times_master_ms,
                           time_solve_subproblems_model_ms=times_sub_ms,
                           objectives=objectives,
                           matrices=matrices,
                           routings=routings,
                           master_model=rm)
end

function _oblivious_reformulation_solve_pricing_problem(rd::RoutingData, dual_values_uncer, dual_value_convexity)
    paths = Dict{Edge{Int}, Vector{Edge}}()

    for demand in demands(rd)
        # Determine the weight matrix to use for the computations:
        #     for d: \sum_{e\in p} dual_{e, d}
        rd.logmessage("Pricing for $demand")
        weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
        for e in edges(rd)
            if dual_values_uncer[demand][e] <= - CPLEX_REDUCED_COST_TOLERANCE
                weight_matrix[src(e), dst(e)] = dual_values_uncer[demand][e]
            end
        end
        weight_matrix .+= maximum(abs.(weight_matrix))

        # Check that at least some weights are nonzero.
        if nnz(weight_matrix) == 0
            # No need for the potentially expensive _check_weight_matrix,
            # as no value in weight_matrix can be below the tolerance by
            # construction (no addition, unlike iterative version of oblivious).
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

    return paths
end
