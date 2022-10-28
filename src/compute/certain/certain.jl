# TODO: column generation for all these functions.

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
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

    # Enforce the timeout on the solver: it will take all the time in this
    # function, more or less.
    if rd.timeout.value > 0
        set_time_limit_sec(rm.model, floor(rd.timeout, Second).value)
    end

    # Solve the problem.
    start = time_ns()
    optimize!(m)

    solve_time = (time_ns() - start) / 1_000_000.

    return RoutingSolution(rd,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=solve_time,
                           time_solve_master_model_ms=solve_time,
                           objectives=objective_value(m),
                           matrices=rd.traffic_matrix,
                           routings=Routing(rd, value.(rm.routing)),
                           master_model=rm)
end

_warn(edge_obj::EdgeWiseObjectiveFunction, sense::String) =
    @warn "The objective function $edge_obj does not support $(sense)imisation. Proceed with caution."
_warn_min(edge_obj::EdgeWiseObjectiveFunction) =
    supports_min(edge_obj) || _warn(edge_obj, "min")
_warn_max(edge_obj::EdgeWiseObjectiveFunction) =
    supports_max(edge_obj) || _warn(edge_obj, "max")

_ensure_objective_compatibility(edge_obj::EdgeWiseObjectiveFunction, ::Union{MinimumTotal, MinimumMaximum}) = _warn_min(edge_obj)
_ensure_objective_compatibility(edge_obj::EdgeWiseObjectiveFunction, ::Union{MaximumTotal, MaximumMinimum}) = _warn_max(edge_obj)

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinimumTotal, MaximumTotal}, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
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
    if agg_obj == MinimumTotal()
        @objective(m, Min, obj)
    else
        @assert agg_obj == MaximumTotal()
        @objective(m, Max, obj)
    end

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Enforce the timeout on the solver: it will take all the time in this
    # function, more or less.
    if rd.timeout.value > 0
        set_time_limit_sec(rm.model, floor(rd.timeout, Second).value)
    end

    # Done!
    start = time_ns()
    optimize!(m)

    solve_time = (time_ns() - start) / 1_000_000.

    return RoutingSolution(rd,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=solve_time,
                           time_solve_master_model_ms=solve_time,
                           objectives=objective_value(m),
                           matrices=rd.traffic_matrix,
                           routings=Routing(rd, value.(rm.routing)),
                           master_model=rm)
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinimumMaximum, MaximumMinimum}, ::FormulationType, ::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    _ensure_objective_compatibility(edge_obj, agg_obj)

    start = time_ns()

    # Create the problem.
    m = _create_model(rd)
    rm = basic_routing_model_unitary(m, rd)

    # Constraints.
    capacity_constraints(rm, rd.traffic_matrix)

    # Objective function. Don't define a lower or an upper bound, in case the
    # individual terms do not have such a bound (like α-fairness).
    @variable(m, obj)
    for e in edges(rd)
        expr = objective_edge_expression(rm, edge_obj, e)
        if agg_obj == MinimumMaximum()
            @constraint(m, obj >= expr)
        else
            @assert agg_obj == MaximumMinimum()
            @constraint(m, obj <= expr)
        end
    end

    if agg_obj == MinimumMaximum()
        @objective(m, Min, obj)
    else
        @assert agg_obj == MaximumMinimum()
        @objective(m, Max, obj)
    end

    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Enforce the timeout on the solver: it will take all the time in this
    # function, more or less.
    if rd.timeout.value > 0
        set_time_limit_sec(rm.model, floor(rd.timeout, Second).value)
    end

    # Done!
    start = time_ns()
    optimize!(m)

    solve_time = (time_ns() - start) / 1_000_000.

    return RoutingSolution(rd,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=solve_time,
                           time_solve_master_model_ms=solve_time,
                           objectives=objective_value(m),
                           matrices=rd.traffic_matrix,
                           routings=Routing(rd, value.(rm.routing)),
                           master_model=rm)
end
