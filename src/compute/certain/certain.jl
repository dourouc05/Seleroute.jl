# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)

    # Objective function.
    @variable(m, mu >= 0)
    rm.mu = mu
    @objective(m, Min, mu)

    # Constraints.
    mu_capacity_constraints(rm, rd.traffic_matrix)

    # Done!
    optimize!(m)
    stop = time_ns()

    return CertainRoutingSolution(rd, rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                                  objective_value(m), rd.traffic_matrix, Routing(rd, value.(rm.routing)), rm)
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, ::MinimumTotal, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)

    # Constraints.
    capacity_constraints(rm, rd.traffic_matrix)

    # Objective function.
    @objective(m, Min, sum(objective_edge_expression(rm, edge_obj, e) for e in edges(rd)))

    # Done!
    optimize!(m)
    stop = time_ns()

    return CertainRoutingSolution(rd, rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                                  objective_value(m), rd.traffic_matrix, Routing(rd, value.(rm.routing)), rm)
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    start = time_ns()

    # Create the problem.
    m = Model(rd.solver)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd)

    # Constraints.
    capacity_constraints(rm, rd.traffic_matrix)

    # Objective function. Don't define a lower bound, in case the individual
    # terms do not have such a lower bound (like α-fairness, even though it
    # does not make much sense to minimise). 
    @variable(m, obj)
    for e in edges(rd)
        @constraint(m, obj >= objective_edge_expression(rm, edge_obj, e))
    end

    @objective(m, Min, obj)

    # Done!
    optimize!(m)
    stop = time_ns()

    return CertainRoutingSolution(rd, rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                                  objective_value(m), rd.traffic_matrix, Routing(rd, value.(rm.routing)), rm)
end
