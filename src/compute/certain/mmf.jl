# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, obj_edge::EdgeWiseObjectiveFunction, ::MinMaxFair, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)
    mu_capacity_constraints(rm, rd.traffic_matrix)

    # Iteratively solve it by adding constraints for each edge.
    last_edge = nothing # last() does not work here (iterator, not a collection).
    for e in edges(rd)
        last_edge = e
    end
    for e in edges(rd)
        # Optimise this objective function.
        @objective(m, Min, objective_edge_expression(rm, obj_edge, e))
        optimize!(m)
        status = termination_status(rm.model)

        # Export if needed.
        _export_lp_if_allowed(rd, m, "lp_$(e)")
        _export_lp_if_failed(rd, status, m, "error_$(e)", "Current problem could not be solved!")

        # Add this value as a requirement for the next iterations (with some numerical leeway).
        # This is not needed for the last iteration.
        if e != last_edge
            ε = 1.e-2 # TODO: Parameter! But that means changing enumerations to types... A good thing, but a lot of refactoring!
            o = objective_value(m) # Must be called before the expression is built!
            # Indeed, an objective expression may require adding variables and constraints.
            expr = objective_edge_expression(rm, obj_edge, e)
            @constraint(m, expr <= (1 + ε) * o)
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
