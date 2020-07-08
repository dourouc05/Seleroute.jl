function master_formulation(rd::RoutingData, ::PathFormulation)
    m = Model(rd.solver)
    set_silent(m)

    rm = basic_routing_model_unitary(m, rd, PathFormulation()) # Includes convexity constraint.

    @variable(m, mu >= 0)
    @objective(m, Min, mu)

    if rd.model_robust_reformulation_traffic_matrices
        @variable(m, dual_alpha[e in edges(rd), k in demands(rd)] <= 0)
        @variable(m, dual_beta[e in edges(rd), e2 in edges(rd)] >= 0)
    else
        @variable(m, dual[e in edges(rd), e2 in edges(rd)] >= 0)
    end

    # Robust uncertainty set.
    if rd.model_robust_reformulation_traffic_matrices
        dual_constraints = Dict{Edge{Int}, Dict{Edge{Int}, ConstraintRef}}(
            e => Dict{Edge{Int}, ConstraintRef}() for e in edges(rd)
        ) # Edge -> demand -> constraint reference.
    end
    for e in edges(rd)
        # Uncertainty set for one edge.
        for d in demands(rd)
            rhs = AffExpr(0.0)
            for p in rd.demand_to_path_ids[d]
                if e in rd.paths_edges[p]
                    add_to_expression!(rhs, rm.routing[d, p])
                end
            end

            if rd.model_robust_reformulation_traffic_matrices
                dual_constraints[e][d] = @constraint(m, rhs + dual_alpha[e, d] <= 0)
                for p in rd.demand_to_path_ids[d]
                    @constraint(m, dual_alpha[e, d] + sum(dual_beta[e, e2] for e2 in rd.paths_edges[p]) >= 0)
                end
            else
                for p in rd.demand_to_path_ids[d]
                    @constraint(m, rhs <= sum(dual[e, e2] for e2 in rd.paths_edges[p]))
                end
            end
        end

        # Relate the main decision variables to the uncertainty sets.
        let dual_var = (rd.model_robust_reformulation_traffic_matrices ? dual_beta : dual)
            rhs = sum(dual_var[e, e2] * get_prop(rd.g, e2, :capacity) for e2 in edges(rd))
            rhs /= get_prop(rd.g, e, :capacity)
            @constraint(m, rhs <= mu)
        end
    end

    if rd.model_robust_reformulation_traffic_matrices
        return RoutingModel(rd, m, UnitaryFlows, mu, rm.routing,
                            dual_alpha=dual_alpha, dual_beta=dual_beta,
                            constraints_uncertainty_set=dual_constraints,
                            constraints_convexity=rm.constraints_convexity)
    else
        return RoutingModel(rd, m, UnitaryFlows, mu, rm.routing, dual=dual,
                            constraints_convexity=rm.constraints_convexity)
    end
end
