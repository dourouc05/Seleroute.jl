# TODO: column generation for all these functions.

function _master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction)
    rm = basic_routing_model_unitary(m, rd)
    capacity_constraints(rm, rd.traffic_matrix)

    @variable(m, τ)
    rm.tau = τ

    return rm
end

function master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MinMaxFair)
    rm = _master_mmf_problem(m, rd, edge_obj)

    @objective(m, Min, rm.tau)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) <= rm.tau)
    rm.constraints_mmf = mmf

    return rm
end

function master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MaxMinFair)
    rm = _master_mmf_problem(m, rd, edge_obj)

    @objective(m, Max, rm.tau)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) >= rm.tau)
    rm.constraints_mmf = mmf

    return rm
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinMaxFair, MaxMinFair}, type::FormulationType, cg::Val, algo::Automatic, unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    # Implementation of the water-filling algorithm for MMF routings (aka MFMF).
    # https://ica1www.epfl.ch/PS_files/LEB3132.pdf
    # https://www.utc.fr/~dnace/recherche/Publication/Inoc03Nace.pdf

    start = time_ns()

    # Create the master problem.
    m = _create_model(rd)
    rm = master_mmf_problem(m, rd, edge_obj, agg_obj)
    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Memorise the evolution of the algorithm.
    n_iter = 0
    n_new_paths = 0
    routings = Routing[]
    objectives = Float64[]
    times_ms = Float64[]
    times_master_ms = Float64[]
    times_sub_ms = Float64[]
    end_status = MOI.OPTIMAL

    # Iteratively solve it by removing τ constraints and fixing the values of
    # the corresponding objectives.
    edges_to_do = Set(edges(rd))
    fixed_objectives = Dict{Edge{Int}, Float64}()

    while length(edges_to_do) > 0
        time_to_solve_master = 0.0
        time_to_solve_sub = 0.0

        start = time_ns()

        # Optimise for this iteration, possibly using column generation.
        # When there is no column generation, this loop only executes once.
        cg_it = 0
        while true
            # Actual optimisation of the master problems.
            start_master = time_ns()
            optimize!(m)
            time_to_solve_master += (time_ns() - start_master) / 1_000_000.

            status = termination_status(m)
            _export_lp_if_allowed(rd, m, "lp_$(n_iter)_$(cg_it)")

            # Handle abnormal cases.
            if status != MOI.OPTIMAL
                # OPTIMAL status is shown with the objective value.
                rd.logmessage("[$(n_iter)] Status: $(status).")

                # Debug infeasibility if need be.
                _debug_infeasibility_mmf(rd, status, n_iter, edge_obj, agg_obj, type, cg, algo, unc, uncparams)

                # Report other problematic codes.
                if status in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED] && n_iter > 0
                    end_status = MOI.ALMOST_OPTIMAL
                    break
                end
            end

            # Perform some column generation if need be, quit otherwise.
            if cg == Val(false)
                break
            end
            @assert typeof(type) <: PathFormulation

            # First, the pricing.
            dual_value_convexity = dual.(rm.constraints_convexity) # Demand -> dual value
            dual_values_capacity = Dict(e => dual(rm.constraints_capacity[e]) for e in edges(rd)) # Edge -> dual value
            dual_values_mmf = Dict(e => dual(rm.constraints_mmf[e]) for e in edges(rd)) # Edge -> dual value

            start_sub = time_ns()
            new_paths = _mmf_solve_pricing_problem(rd, dual_values_capacity, dual_values_mmf, dual_value_convexity)
            time_to_solve_sub += (time_ns() - start_sub) / 1_000_000.
            n_new_paths += length(new_paths)

            # Add the new columns to the formulation.
            for (d, path) in new_paths
                # Add the new columns to the data object and retrieve a path ID.
                path_id = add_path!(rm, d, path)

                # Get your hands dirty in the formulation.
                # - Create a new variable for this path, add it into the
                #   convexity and capacity constraints.
                add_routing_var!(rm, demand, path_id, constraint_capacity=true)

                # TODO: normalised way of having traffic matrices in rd? Are graph attributes forbidden/unused everywhere? What about passing them as arguments, as helpers in knowncapacities?
                dd = rd.traffic_matrix[d]
                for e in path
                    # - MMF constraint.
                    ce = capacity(rd, e)
                    set_normalized_coefficient(rm.constraints_mmf[e], rm.routing[d, path_id], dd / ce)
                end
            end

            rd.logmessage("[$(n_iter) | $(cg_it)] Added $(length(new_paths)) new paths.")
            cg_it += 1

            if length(new_paths) == 0
                break
            end
        end

        # Report the values for this iteration.
        master_τ = objective_value(m)
        rd.logmessage("[$(n_iter)] Status: $(status). Value: $(master_τ)")
        push!(times_master_ms, time_to_solve_master)
        push!(times_sub_ms, time_to_solve_sub)
        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, master_τ)

        # Find edges to fix.
        new_edges = _mmf_find_edges_to_fix(rm, master_τ, edges_to_do, agg_obj)
        rd.logmessage("  Number of variables to fix: $(length(new_edges)).")

        # Debugging when no edge met the conditions.
        if length(new_edges) == 0
            if n_iter == 0
                # No variable can be fixed right now, there must be a problem!
                @warn "No variable can be fixed at the first iteration, " *
                      "the MMF process stalls."
            else
                # Every link seems to be saturated, end now.
                rd.logmessage("[$(n_iter)] All links are saturated.")
            end
        end

        # Add the new constraints for the next iteration.
        for (e, value) in new_edges
            # Maintain the data structures.
            fixed_objectives[e] = value
            pop!(edges_to_do, e)

            # Modify the master to indicate that the value is now known.
            _mmf_fix_edge(rm, e, value, agg_obj)
        end

        rd.logmessage("  Edges not yet saturated: $(length(edges_to_do)). Total edges saturated: $(length(fixed_objectives))")

        # Log a few things at the end of the iteration. Stop if no new edge was
        # found.
        n_iter += 1
        push!(times_ms, (time_ns() - start) / 1_000_000.)

        if length(new_edges) == 0
            # The user is already warned.
            break
        end
    end

    # Not necessary to optimise one last time: the remaining edges were fixed
    # to the current value in the master (i.e. the last iteration just added
    # redundant constraints).

    return RoutingSolution(rd, result=end_status,
                           n_columns=n_new_paths,
                           n_columns_master=n_new_paths,
                           n_columns_subproblems=0,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_create_subproblems_model_ms=0.0, # No subproblem, only shortest paths.
                           time_solve_ms=times_ms,
                           time_solve_master_model_ms=times_master_ms,
                           time_solve_subproblems_model_ms=times_sub_ms,
                           objectives=objectives,
                           matrices=rd.traffic_matrix,
                           routings=routings,
                           master_model=rm)
end

function _mmf_solve_pricing_problem(rd::RoutingData, dual_values_capacity, dual_values_mmf, dual_value_convexity)
    paths = Dict{Edge{Int}, Vector{Edge}}()

    # Determine the weight matrix to use for the computations:
    #     \sum_{e\in p} dualcapa_{e} + dualmmf_{e} / capa_e
    weight_matrix = spzeros(Float64, n_nodes(rd), n_nodes(rd))
    for e in edges(rd)
        value = dual_values_capacity[e] + dual_values_mmf[e] / capacity(rd, e)
        if value <= - CPLEX_REDUCED_COST_TOLERANCE
            weight_matrix[src(e), dst(e)] = value
        end
    end
    weight_matrix .+= maximum(abs.(weight_matrix))

    # Check that at least some weights are nonzero.
    if nnz(weight_matrix) == 0
        # No need for the potentially expensive _check_weight_matrix,
        # as no value in weight_matrix can be below the tolerance by
        # construction (no addition, unlike iterative version of oblivious).
        return paths
    end

    # Perform the pricing for each demand.
    for demand in demands(rd)
        rd.logmessage("Pricing for $demand")

        # Compute the shortest path for this demand.
        state = dijkstra_shortest_paths(rd.g, src(demand), weight_matrix)

        # Is there a solution? Has this path a negative reduced cost?
        if all(state.parents .== 0)
            continue
        end
        reduced_cost = - state.dists[dst(demand)] * rd.traffic_matrix[demand] - dual_value_convexity[demand]
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

function _mmf_find_edges_to_fix(rm::RoutingModel, master_τ::Float64, edges_to_do, agg_obj)
    # Check which constraints are satisfied at equality (i.e. nonzero dual
    # value, as the constraint is either ≥ or ≤).
    new_edges = Dict{Edge{Int}, Float64}()

    if has_duals(rm.model)
        for e in edges_to_do
            if - agg_obj.ε <= dual(rm.constraints_mmf[e]) <= agg_obj.ε
                new_edges[e] = master_τ
            end
        end
    end

    if length(new_edges) == 0
        # If no edge goes through the dual variable test, check the actual
        # values (this may be due to the way the objective function is
        # modelled, like Fortz-Thorup, or if the solver doesn't give duals).
        for e in edges_to_do
            if - agg_obj.ε <= value(rm.constraints_mmf[e]) <= agg_obj.ε
                new_edges[e] = master_τ
            end
        end
    end

    return new_edges
end

function _mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, rhs::Float64)
    rm.data.logmessage("  Fixing $e to $value ≈ $rhs (relaxed value)")
    set_normalized_coefficient(rm.constraints_mmf[e], rm.tau, 0)
    set_normalized_rhs(rm.constraints_mmf[e], rhs)
end

_mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, agg_obj::MinMaxFair) =
    _mmf_fix_edge(rm, e, value, value * (1 + agg_obj.ε))

_mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, agg_obj::MaxMinFair) =
    _mmf_fix_edge(rm, e, value, value * (1 - agg_obj.ε))

# Debug infeasiblity. Implement the main logic here. Throw an error in case of
# infeasibility.
function _debug_infeasibility_mmf(rd::RoutingData, status::MOI.TerminationStatusCode, n_iter::Int,
                                  edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinMaxFair, MaxMinFair},
                                  type::FormulationType, cg::Val, algo::Automatic,
                                  unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    if status in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED] && n_iter == 0
        rd.logmessage("[$(n_iter)] This iteration of MMF yielded an infeasible optimisation program. ")
        d = _debug_infeasibility_mmf(rd, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
        if d !== nothing
            rd.logmessage("[$(n_iter)] The source of infeasibility could be automatically detected: ")
            rd.logmessage("[$(n_iter)] $d")
        else
            rd.logmessage("[$(n_iter)] The source of infeasibility could not be automatically detected.")
        end

        # Exit with an error.
        _export_lp_if_failed(rd, status, m, "error_$(n_iter)", "Current problem could not be solved!")
    end
end

_debug_infeasibility_mmf(::RoutingData, ::EdgeWiseObjectiveFunction,
                         ::MinMaxFair, ::FormulationType, ::Val, ::Automatic,
                         ::NoUncertaintyHandling, ::NoUncertainty) = nothing

function _debug_infeasibility_mmf(rd::RoutingData, ::KleinrockLoad,
                                  ::MinMaxFair, type::FormulationType,
                                  ::Val{false}, ::Automatic,
                                  ::NoUncertaintyHandling, ::NoUncertainty)
    # Main source of infeasibility: impossible to have links with a load less
    # than 100%.
    # Technique: minimise the maximum load (i.e. not Kleinrock), check if it
    # is 100%.
    r = compute_routing(rd, Load(), MinimumMaximum(), type, Val(false),
                        Automatic(), NoUncertaintyHandling(), NoUncertainty())
    if compute_max_load(r) >= 0.99999
        return "This network cannot withstand this traffic matrix: " *
               "its maximum load is 100% or more (i.e. the most loaded link is " *
               "loaded at 100%, when minimising this maximum load). " *
               "Kleinrock function associates an infinite penalty to links " *
               "with such a high load."
    end
    return nothing
end
