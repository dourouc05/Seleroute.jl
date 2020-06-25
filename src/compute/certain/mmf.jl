# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinMaxFair, MaxMinFair}, type::FormulationType, cg::Val{false}, algo::Automatic, unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    # Based on https://onlinelibrary.wiley.com/doi/abs/10.1002/ett.1047.
    start = time_ns()

    # Create the master problem.
    m = Model(rd.solver)
    set_silent(m)
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

    # Memorise the evolution of the algorithm.
    n_iter = 0
    routings = Routing[]
    objectives = Float64[]

    # Iteratively solve it by removing τ constraints and fixing the values of
    # the corresponding objectives.
    edges_to_do = Set(edges(rd))
    edges_done = Set{Edge{Int}}()
    fixed_objectives = Dict{Edge{Int}, Float64}() # Its keys must correspond to edges_done. TODO: remove edges_done.

    while length(edges_to_do) > 0
        # Optimise for this iteration.
        optimize!(m)
        status = termination_status(m)

        if status != MOI.OPTIMAL
            # OPTIMAL status is shown with the objective value.
            rd.logmessage("[$(n_iter)] Status: $(status).")
        end

        # Debug infeasibility if need be.
        if status == MOI.INFEASIBLE
            rd.logmessage("This iteration of MMF yielded an infeasible optimisation program. ")
            d = _debug_infeasibility_mmf(rd, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
            if d !== nothing
                rd.logmessage("The source of infeasibility could be automaticall detected: ")
                rd.logmessage(d)
            else
                rd.logmessage("The source of infeasibility could not be automatically detected.")
            end
        end

        # Export if needed.
        _export_lp_if_allowed(rd, m, "lp_$(n_iter)")
        _export_lp_if_failed(rd, status, m, "C:\\Users\\Thibaut\\.julia\\dev\\Seleroute\\examples\\" * "error_$(n_iter)", "Current problem could not be solved!")

        # Report the values for this iteration. Any of these lines will fail
        # in case of infeasibility.
        master_τ = objective_value(m)
        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, master_τ)

        println("[$(n_iter)] Status: $(status). Value: $(master_τ)")

        # Solve the subproblem to determine which variables should now be fixed.
        # TODO: reuse the subproblem to avoid rebuilding it at every iteration.
        new_edges = Dict{Edge{Int}, Float64}()
        for e in edges_to_do
            # Create the subproblem.
            s_m = Model(rd.solver)
            set_silent(s_m)
            s_rm = basic_routing_model_unitary(s_m, rd)
            capacity_constraints(s_rm, rd.traffic_matrix)

            # With respect to the theoretical algorithm, don't use the variable
            # γ, directly put the expression in the objective.
            obj = objective_edge_expression(s_rm, edge_obj, e)
            if typeof(agg_obj) == MinMaxFair
                @objective(s_m, Min, obj)
            else
                @assert typeof(agg_obj) == MaxMinFair
                @objective(s_m, Max, obj)
            end

            # Add constraints for the previously fixed edges.
            # Instead of equalities, to slightly expand the feasible area and
            # avoid numerical problems, replace by inequalities.
            for e2 in edges_done
                e2_expr = objective_edge_expression(s_rm, edge_obj, e2)
                if iszero(agg_obj.ε)
                    @constraint(s_m, e2_expr == fixed_objectives[e2])
                else
                    @constraint(s_m, e2_expr >= fixed_objectives[e2] * (1 - agg_obj.ε))
                    @constraint(s_m, e2_expr <= fixed_objectives[e2] * (1 + agg_obj.ε))
                end
            end

            # Add constraints for the still unfixed edges.
            if typeof(agg_obj) == MinMaxFair
                for e2 in edges_to_do
                    if e2 != e
                        @constraint(s_m, objective_edge_expression(s_rm, edge_obj, e2) <= master_τ * (1 + agg_obj.ε))
                    end
                end
            else
                @assert typeof(agg_obj) == MaxMinFair

                for e2 in edges_to_do
                    if e2 != e
                        @constraint(s_m, objective_edge_expression(s_rm, edge_obj, e2) >= master_τ * (1 - agg_obj.ε))
                    end
                end
            end

            # Solve the subproblem.
            optimize!(s_m)
            s_status = termination_status(s_m)

            if s_status != MOI.OPTIMAL
                println("[$(n_iter)][$e] Status: $(s_status).")
            end

            # Export if needed.
            _export_lp_if_allowed(rd, s_m, "lp_$(n_iter)_$(e)")
            _export_lp_if_failed(rd, s_status, s_m, "C:\\Users\\Thibaut\\.julia\\dev\\Seleroute\\examples\\" * "error_$(n_iter)_$(src(e))_$(dst(e))", "Current subproblem could not be solved!")

            short(x) = 0.001 * round(Int, 1000 * x)
            println("[$(n_iter)][$e] Status: $(s_status). Value: $(short(objective_value(s_m))). Delta with master: $(short(abs(objective_value(s_m) - master_τ))).")

            # Determine whether the value of the objective should be fixed.
            if abs(objective_value(s_m) - master_τ) < agg_obj.ε # TODO: another parameter than ε?
                new_edges[e] = master_τ
                println("[$(n_iter)][$e] => Fixed ($(length(edges_to_do)) to do, $(length(edges_done)) done)")
            end
        end

        if length(new_edges) == 0
            @warn "No new variable has been fixed in this iteration, the MMF " *
                  "iterative process is stalling. Consider using a lower " *
                  "precision."
        end

        for (e, value) in new_edges
            # Maintain the data structures.
            fixed_objectives[e] = value
            push!(edges_done, e)
            pop!(edges_to_do, e)

            # Modify the master to indicate that the value is now known.
            set_normalized_coefficient(mmf[e], τ, 0)
            rhs = if typeof(agg_obj) == MinMaxFair
                fixed_objectives[e] * (1 + agg_obj.ε)
            else
                @assert typeof(agg_obj) == MaxMinFair
                fixed_objectives[e] * (1 - agg_obj.ε)
            end
            set_normalized_rhs(mmf[e], rhs)
        end

        n_iter += 1
        println()
    end

    # Not necessary to optimise one last time: the remaining edges were fixed
    # to the current value in the master (i.e. the last iteration just added
    # redundant constraints).

    stop = time_ns()

    return RoutingSolution(rd, n_iter, 1, 0, 0,
                           rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                           objectives, [rd.traffic_matrix], routings, rm)
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
