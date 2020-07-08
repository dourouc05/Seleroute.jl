# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinMaxFair, MaxMinFair}, type::FormulationType, cg::Val{false}, algo::Automatic, unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    # Implementation of the water-filling algorithm for MMF routings (aka MFMF).
    # https://ica1www.epfl.ch/PS_files/LEB3132.pdf
    # https://www.utc.fr/~dnace/recherche/Publication/Inoc03Nace.pdf

    start = time_ns()

    # Create the master problem.
    m = Model(rd.solver)
    set_silent(m) # TODO: add a parameter for this. And also a helper method to create the solver?
    rm = basic_routing_model_unitary(m, rd)
    capacity_constraints(rm, rd.traffic_matrix)

    @variable(m, τ)
    # TODO: warnings for objectives.
    if typeof(agg_obj) == MinMaxFair
        @objective(m, Min, τ)
        @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) <= τ)
    else
        @assert typeof(agg_obj) == MaxMinFair
        @objective(m, Max, τ)
        @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) >= τ)
    end

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Memorise the evolution of the algorithm.
    n_iter = 0
    routings = Routing[]
    objectives = Float64[]
    times_ms = Float64[]

    # Iteratively solve it by removing τ constraints and fixing the values of
    # the corresponding objectives.
    edges_to_do = Set(edges(rd))
    fixed_objectives = Dict{Edge{Int}, Float64}()

    end_status = MOI.OPTIMAL

    while length(edges_to_do) > 0
        start = time_ns()

        # Optimise for this iteration.
        optimize!(m)
        status = termination_status(m)
        _export_lp_if_allowed(rd, m, "lp_$(n_iter)")

        # Handle abnormal cases.
        if status != MOI.OPTIMAL
            # OPTIMAL status is shown with the objective value.
            rd.logmessage("[$(n_iter)] Status: $(status).")

            # Debug infeasibility at the first round.
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

            # Report other problematic codes.
            if status in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED] && n_iter > 0
                end_status = MOI.ALMOST_OPTIMAL
                break
            end
            # if status != MOI.OPTIMAL && end_status == MOI.OPTIMAL
            #     end_status = status
            #     break
            # end
        end

        # Report the values for this iteration.
        master_τ = objective_value(m)
        rd.logmessage("[$(n_iter)] Status: $(status). Value: $(master_τ)")
        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, master_τ)

        # Check which constraints are satisfied at equality (i.e. nonzero dual
        # value, as the constraint is either ≥ or ≤).
        new_edges = Dict{Edge{Int}, Float64}()

        if has_duals(m)
            for e in edges_to_do
                if - agg_obj.ε <= dual(mmf[e]) <= agg_obj.ε
                    new_edges[e] = master_τ
                end
            end
        end

        if length(new_edges) == 0
            # If no edge goes through the dual variable test, check the actual
            # values (this may be due to the way the objective function is
            # modelled, like Fortz-Thorup, or if the solver doesn't give duals).
            for e in edges_to_do
                if - agg_obj.ε <= value(mmf[e]) <= agg_obj.ε
                    new_edges[e] = master_τ
                end
            end
        end

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
            set_normalized_coefficient(mmf[e], τ, 0)
            rhs = value * if typeof(agg_obj) == MinMaxFair
                1 + agg_obj.ε
            else
                @assert typeof(agg_obj) == MaxMinFair
                1 - agg_obj.ε
            end
            rd.logmessage("  Fixing $e to $master_τ ≈ $rhs (relaxed value)")
            set_normalized_rhs(mmf[e], rhs)
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
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=times_ms,
                           objectives=objectives,
                           matrices=rd.traffic_matrix,
                           routings=routings,
                           master_model=rm)
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
