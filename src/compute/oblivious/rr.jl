master_formulation(rd::RoutingData, ::Any) =
    error("Not implemented: $(rd.model_type)")

# TODO: like _iter, add solve_master_problem that in turn calls this.

function get_traffic_matrices(rm::RoutingModel)
    rd = rm.data
    dual_constraints = rm.constraints_uncertainty_set

    l_tm = Set{Dict{Edge{Int}, Float64}}()
    for e in edges(rd)
        push!(l_tm, Dict{Edge{Int}, Float64}(k => JuMP.dual(dual_constraints[e][k]) for k in keys(dual_constraints[e])))
    end
    return collect(l_tm)
end

function compute_routing(rd::RoutingData, ::Load, ::MinimumMaximum, formulation::FormulationType, ::Val{false}, ::DualReformulation, ::ObliviousUncertainty, ::UncertainDemand)
    start = time_ns()
    rm = master_formulation(rd, formulation)

    # Solve the problem and generate the output data structure.
    optimize!(rm.model)
    rd.logmessage(objective_value(rm.model))
    status = termination_status(rm.model)

    # Export if needed.
    _export_lp_if_allowed(rd, rm.model, "lp_master")
    _export_lp_if_failed(rd, status, rm.model, "error_master", "Subproblem could not be solved!")

    # Retrieve the traffic matrices, if asked.
    tms = if rd.model_robust_reformulation_traffic_matrices
        get_traffic_matrices(rm)
    else
        Dict{Edge{Int}, Float64}[]
    end

    stop = time_ns()

    # TODO: Export things like the normal case.

    return RoutingSolution(rd, 1, length(tms), 0, 0,
                           rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                           [value(rm.mu)], tms, Routing[Routing(rd, value.(rm.routing))], rm)
end
