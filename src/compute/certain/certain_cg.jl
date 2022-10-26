function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MinimumTotal, ::PathFormulation, ::Val{true}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    _ensure_objective_compatibility(edge_obj, agg_obj)

    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)

    # Constraints.
    capacity_constraints(rm, rd.traffic_matrix)

    # Objective function.
    obj = sum(objective_edge_expression(rm, edge_obj, e) for e in edges(rd))
    @objective(m, Min, obj)

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Solve this by column generation.
    routings = Routing[]
    objectives = Float64[]
    times_ms = Float64[]
    times_master_ms = Float64[]
    times_sub_ms = Float64[]
    n_new_paths = 0
    n_iter = 0

    while true
        start = time_ns()

        # Optimise for the current iteration.
        start_master = time_ns()
        optimize!(m)
        end_master = time_ns()

        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, objective_value(m))

        # First, the pricing.
        dual_value_convexity = dual.(rm.constraints_convexity) # Demand -> dual value
        dual_values_capacity = Dict(e => dual(rm.constraints_capacity[e]) for e in edges(rd)) # Edge -> dual value

        start_sub = time_ns()
        new_paths = _mintotload_solve_pricing_problem_master(rd, dual_values_capacity, dual_value_convexity)
        end_sub = time_ns()
        n_new_paths += length(new_paths)

        # Add the new columns to the formulation.
        for (d, path) in new_paths
            # Add the new columns to the data object and retrieve a path ID.
            path_id = add_path!(rm, d, path)

            # Get your hands dirty in the formulation.
            # - Create a new variable for this path, add it into the
            #   convexity and capacity constraints.
            add_routing_var!(rm, demand, path_id, constraint_capacity=true)
        end

        rd.logmessage("[$(n_iter)] Added $(length(new_paths)) new paths.")
        n_iter += 1
        push!(times_ms, (time_ns() - start) / 1_000_000.)
        push!(times_master_ms, (end_master - start_master) / 1_000_000.)
        push!(times_sub_ms, (end_sub - start_sub) / 1_000_000.)

        if length(new_paths) == 0
            break
        end
    end

    return RoutingSolution(rd,
                           n_columns=n_new_paths,
                           n_columns_master=n_new_paths,
                           n_columns_subproblems=0,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_create_subproblems_model_ms=0.0, # No subproblem, only shortest paths.
                           time_solve_ms=times_ms,
                           time_solve_master_model_ms=times_master_ms,
                           time_solve_subproblems_model_ms=times_sub_ms,
                           objectives=objective_value(m),
                           matrices=rd.traffic_matrix,
                           routings=Routing(rd, value.(rm.routing)),
                           master_model=rm)
end

function _mintotload_solve_pricing_problem_master(rd::RoutingData,
                                                  dual_values_capacity,
                                                  dual_value_convexity)
    paths = Dict{Edge{Int}, Vector{Edge}}()

    for demand in demands(rd)
        # Determine the weight matrix to use for the computations:
        #     for e: \sum_{D\in\Delta} dual_{e, d_k} d_k
        rd.logmessage("Pricing for $demand")
        weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
        for e in edges(rd)
            value = 1.0 / capacity(rd, e) - rd.traffic_matrix[demand] * dual_values_capacity[e]
            if value >= - CPLEX_REDUCED_COST_TOLERANCE
                weight_matrix[src(e), dst(e)] = value
            end
        end

        # Check that at least some weights are nonzero.
        if nnz(weight_matrix) == 0
            # No need for the potentially expensive _check_weight_matrix,
            # as no value in weight_matrix can be below the tolerance by
            # construction (no addition, unlike iterative version of oblivious).
            return paths
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

function compute_routing(rd::RoutingData, ::EdgeWiseObjectiveFunction, ::MinimumTotal, ::PathFormulation, ::Val{true}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    start = time_ns()

    # Create the problem.
    m = _create_model(rd)
    rm = basic_routing_model_unitary(m, rd)

    # Objective function.
    @variable(m, mu >= 0)
    rm.mu = mu
    @objective(m, Min, mu)

    # Constraints.
    mu_capacity_constraints(rm, rd.traffic_matrix)

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Solve this by column generation.
    routings = Routing[]
    objectives = Float64[]
    times_ms = Float64[]
    times_master_ms = Float64[]
    times_sub_ms = Float64[]
    n_new_paths = 0
    n_iter = 0

    while true
        start = time_ns()

        # Optimise for the current iteration.
        start_master = time_ns()
        optimize!(m)
        end_master = time_ns()

        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, objective_value(m))

        # First, the pricing.
        dual_value_convexity = dual.(rm.constraints_convexity) # Demand -> dual value
        dual_values_capacity = Dict(e => dual(rm.constraints_capacity[e]) for e in edges(rd)) # Edge -> dual value

        start_sub = time_ns()
        new_paths = _minmaxload_solve_pricing_problem_master(rd, dual_values_capacity, dual_value_convexity)
        end_sub = time_ns()
        n_new_paths += length(new_paths)

        # Add the new columns to the formulation.
        for (d, path) in new_paths
            # Add the new columns to the data object and retrieve a path ID.
            path_id = add_path!(rm, d, path)

            # Get your hands dirty in the formulation.
            # - Create a new variable for this path, add it into the
            #   convexity and capacity constraints.
            add_routing_var!(rm, demand, path_id, constraint_capacity=true)
        end

        rd.logmessage("[$(n_iter)] Added $(length(new_paths)) new paths.")
        n_iter += 1
        push!(times_ms, (time_ns() - start) / 1_000_000.)
        push!(times_master_ms, (end_master - start_master) / 1_000_000.)
        push!(times_sub_ms, (end_sub - start_sub) / 1_000_000.)

        if length(new_paths) == 0
            break
        end
    end

    return RoutingSolution(rd,
                           n_columns=n_new_paths,
                           n_columns_master=n_new_paths,
                           n_columns_subproblems=0,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_create_subproblems_model_ms=0.0, # No subproblem, only shortest paths.
                           time_solve_ms=times_ms,
                           time_solve_master_model_ms=times_master_ms,
                           time_solve_subproblems_model_ms=times_sub_ms,
                           objectives=objective_value(m),
                           matrices=rd.traffic_matrix,
                           routings=Routing(rd, value.(rm.routing)),
                           master_model=rm)
end

function _minmaxload_solve_pricing_problem_master(rd::RoutingData,
                                                  dual_values_capacity,
                                                  dual_value_convexity)
    paths = Dict{Edge{Int}, Vector{Edge}}()

    for demand in demands(rd)
        # Determine the weight matrix to use for the computations:
        #     for e: \sum_{D\in\Delta} dual_{e, d_k} d_k
        rd.logmessage("Pricing for $demand")
        weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
        for e in edges(rd)
            value = 1.0 / capacity(rd, e) - rd.traffic_matrix[demand] * dual_values_capacity[e]
            if value >= - CPLEX_REDUCED_COST_TOLERANCE
                weight_matrix[src(e), dst(e)] = value
            end
        end

        # Check that at least some weights are nonzero.
        if nnz(weight_matrix) == 0
            # No need for the potentially expensive _check_weight_matrix,
            # as no value in weight_matrix can be below the tolerance by
            # construction (no addition, unlike iterative version of oblivious).
            return paths
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
