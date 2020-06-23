# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MinMaxFair, type::FormulationType, cg::Val{false}, algo::Automatic, unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    # Based on https://onlinelibrary.wiley.com/doi/abs/10.1002/ett.1047.
    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)
    capacity_constraints(rm, rd.traffic_matrix)

    # Find the last edge.
    last_edge = nothing # last() does not work here (iterator, not a collection).
    for e in edges(rd)
        last_edge = e
    end

    # Iteratively solve it by adding constraints for each edge.
    @variable(m, τ)
    @objective(m, Min, τ)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) <= τ)

    # TODO: memorise all routings, at each iteration? 
    for e in edges(rd)
        # Optimise for this iteration.
        optimize!(m)
        status = termination_status(rm.model)

        # Debug infeasibility if need be.
        println(status)
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
        _export_lp_if_allowed(rd, m, "lp_$(e)")
        _export_lp_if_failed(rd, status, m, "error_$(e)", "Current problem could not be solved!")

        # Add this value as a requirement for the next iterations (with some
        # numerical leeway). This is not needed for the last iteration.
        if e != last_edge
            # Replace "expr_e ≤ τ" (τ being a variable) by "expr_e ≤ obj"
            # (with obj constant).
            set_normalized_rhs(mmf[e], objective_value(m) * (1 + agg_obj.ε))
            set_normalized_coefficient(mmf[e], τ, 0)
        end
    end
    # No need to start the solver one last time, the last solution is the right one.

    # Done, at last!
    stop = time_ns()

    return CertainRoutingSolution(rd, rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                                  NaN, rd.traffic_matrix, Routing(rd, value.(rm.routing)), rm)
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
