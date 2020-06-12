# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MinMaxFair, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
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
    @variable(m, τ >= 0)
    @objective(m, Min, τ)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) <= τ)

    for e in edges(rd)
        # Optimise for this iteration.
        optimize!(m)
        status = termination_status(rm.model)

        # Export if needed.
        _export_lp_if_allowed(rd, m, "lp_$(e)")
        _export_lp_if_failed(rd, status, m, "error_$(e)", "Current problem could not be solved!")

        # Add this value as a requirement for the next iterations (with some numerical leeway).
        # This is not needed for the last iteration.
        if e != last_edge
            set_normalized_rhs(mmf[e], objective_value(m) * (1 + agg_obj.ε))
            set_normalized_coefficient(mmf[e], τ, 0)
        end
    end

    # No need to start the solver one last time, the last solution is the right one.
    # If there were one more solver call, it would have to be without any objective function,
    # but with the constraint corresponding to the last solver call.

    # Done, at last!
    stop = time_ns()

    return CertainRoutingSolution(rd, rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                                  NaN, rd.traffic_matrix, Routing(rd, value.(rm.routing)), rm)
end
