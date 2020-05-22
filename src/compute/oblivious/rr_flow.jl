function master_formulation(rd::RoutingData, type::FlowFormulation)
    m = Model(rd.solver)
    set_silent(m)

    rm = basic_routing_model_unitary(m, rd, type) # Includes flow-conservation constraints.

    @variable(m, mu >= 0)
    @objective(m, Min, mu)

    @variable(m, dual_alpha[e in edges(rd), d in demands(rd), v in vertices(rd)])
    @variable(m, dual_beta[e in edges(rd), e2 in edges(rd)] >= 0)

    # Robust uncertainty sets (one per edge).
    if rd.model_robust_reformulation_traffic_matrices
        dual_constraints = Dict{Edge{Int}, Dict{Edge{Int}, ConstraintRef}}(
            e => Dict{Edge{Int}, ConstraintRef}() for e in edges(rd)
        ) # Edge -> demand -> constraint reference.
    end
    for e in edges(rd)
        for d in demands(rd)
            # Dual constraint from the flow variables.
            for e2 in edges(rd)
                @constraint(m, dual_beta[e, e2] + dual_alpha[e, d, dst(e2)] - dual_alpha[e, d, src(e2)] >= 0)
            end

            # Dual constraint from the demand variables.
            c = @constraint(m, dual_alpha[e, d, src(d)] - dual_alpha[e, d, dst(d)] >= rm.routing[d, e])
            if rd.model_robust_reformulation_traffic_matrices
                dual_constraints[e][d] = c
            end
        end

        # Relate the main decision variables to the uncertainty sets.
        lhs = sum(dual_beta[e, e2] * get_prop(rd.g, e2, :capacity) for e2 in edges(rd))
        @constraint(m, lhs / get_prop(rd.g, e, :capacity) <= mu)
    end

    return RoutingModel(rd, m, UnitaryFlows, mu, rm.routing, dual_alpha=dual_alpha, dual_beta=dual_beta)
end
