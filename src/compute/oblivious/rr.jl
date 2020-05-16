master_formulation(rd::RoutingData, solver, ::Val) =
    error("Not implemented: $(rd.model_type)")

# TODO: like _iter, add solve_master_problem that in turn calls this.

function get_traffic_matrices(rm::RoutingModel)
    rd = rm.data
    dual_constraints = rm.constraints_uncertainty_set

    l_tm = Set{Dict{Edge, Float64}}()
    for e in edges(rd)
        push!(l_tm, Dict{Edge, Float64}(k => JuMP.dual(dual_constraints[e][k]) for k in keys(dual_constraints[e])))
    end
    return collect(l_tm)
end

function compute_routing(rd::RoutingData, ::Val{Load}, ::Val{MinimumMaximum}, formulation::Val, ::Val{false}, ::Val{ObliviousUncertainty}, ::Val{DualReformulation}, ::Val{UncertainDemand})
    start = time_ns()

    rm = master_formulation(rd, solver, formulation)
    if export_lp
        write_LP("$(rd.output_folder)/master.lp", rm.model)
    end

    # Solve the problem and generate the output data structure.
    optimize!(rm.model)
    rd.logmessage(objective_value(rm.model))

    # Retrieve the traffic matrices, if asked.
    tms = if rd.model_robust_reformulation_traffic_matrices
        get_traffic_matrices(rm)
    else
        Dict{Edge, Float64}[]
    end

    stop = time_ns()

    # TODO: Export things like the normal case.

    return RoutingSolution(rd, 1, 1, 1, 0,
                            rd.time_precompute_ms, (stop - start) / 1_000_000., 0.0,
                            [value(rm.mu)], tms, Routing[Routing(rd, value.(routing))], rm.rm)
end
