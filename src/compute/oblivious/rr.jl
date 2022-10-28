master_formulation(rd::RoutingData, ::Any) =
    error("Not implemented: $(rd.model_type)")

# TODO: like _iter, add solve_master_problem that in turn calls this.

function get_traffic_matrices(rm::RoutingModel)
    rd = rm.data
    dual_constraints = rm.constraints_uncertainty_set

    l_tm = Set{Dict{Edge{Int}, Float64}}()
    for e in edges(rd)
        push!(l_tm, Dict{Edge{Int}, Float64}(k => dual(dual_constraints[e][k]) for k in keys(dual_constraints[e])))
    end
    return collect(l_tm)
end

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, formulation::FormulationType, ::Val{false}, ::DualReformulation, ::ObliviousUncertainty, ::UncertainDemand)
    start = time_ns()
    rm = master_formulation(rd, formulation)
    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Enforce the timeout on the solver: it will take more time in this
    # function than modelling (but modelling can be expensive).
    result = MOI.OPTIMAL
    if rd.timeout.value > 0
        set_time_limit_sec(rm.model, floor(rd.timeout, Second).value)

        if Nanosecond(time_ns() - start) >= rd.timeout
            result = MOI.TIME_LIMIT
        end
    end

    # Maybe the formulation timed out.
    if isnothing(rm.routing)
        result = MOI.TIME_LIMIT
    end

    # Solve the problem.
    time_solve_master = 0.0
    if result != MOI.TIME_LIMIT
        start_master = time_ns()
        optimize!(rm.model)
        time_solve_master = (time_ns() - start_master) / 1_000_000.
        result = termination_status(rm.model)
    end
    
    # Finalise the process.
    matrices = Dict{Edge{Int}, Float64}[]
    objectives = Float64[]
    routings = Routing[]
    if result != MOI.TIME_LIMIT
        # Generate the output data structure.
        rd.logmessage(objective_value(rm.model))
        push!(objectives, objective_value(rm.model))
        push!(routings, Routing(rd, value.(rm.routing)))

        # Export if needed.
        _export_lp_if_allowed(rd, rm.model, "lp_master")
        _export_lp_if_failed(rd, result, rm.model, "error_master", "Subproblem could not be solved!")

        # Retrieve the traffic matrices, if asked.
        if rd.model_robust_reformulation_traffic_matrices
            matrices = get_traffic_matrices(rm)
        end
    end

    stop = time_ns()

    # TODO: Export things like the normal case.

    return RoutingSolution(rd,
                           result=result,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=(stop - start) / 1_000_000.,
                           time_solve_master_model_ms=time_solve_master,
                           objectives=objectives,
                           matrices=matrices,
                           routings=routings,
                           master_model=rm)
end
