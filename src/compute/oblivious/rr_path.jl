function master_formulation(rd::RoutingData, ::PathFormulation)
    # Enforce the timeout based on this starting time. This is not super
    # precise, as start is computed when starting modelling, i.e. not when the
    # user expects it (call to compute_routing), but quite close (not much to
    # do before creating the formulation).
    # Modelling can be quite expensive for this formulation, due to O(n³)
    # variables and constraints.
    start = time_ns()

    # Start creating the model.
    m = _create_model(rd)
    rm = basic_routing_model_unitary(m, rd, PathFormulation()) # Includes convexity constraint.

    @variable(m, mu >= 0)
    @objective(m, Min, mu)

    if rd.model_simplifications
        @variable(m, dual_alpha[e in edges(rd), k in demands(rd)] <= 0)
        @variable(m, dual_beta[e in edges(rd), e2 in edges(rd)] >= 0)
    else
        @variable(m, dual[e in edges(rd), e2 in edges(rd)] >= 0)
    end

    # Enforce the timeout.
    if rd.timeout.value > 0 && Nanosecond(time_ns() - start) >= rd.timeout
        return RoutingModel(rd, m, UnitaryFlows, nothing)
    end

    # Robust uncertainty set.
    if rd.model_simplifications
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

            if rd.model_simplifications
                dual_constraints[e][d] = @constraint(m, rhs + dual_alpha[e, d] <= 0)
                for p in rd.demand_to_path_ids[d]
                    @constraint(m, dual_alpha[e, d] + sum(dual_beta[e, e2] for e2 in rd.paths_edges[p]) >= 0)
                end
            else
                for p in rd.demand_to_path_ids[d]
                    @constraint(m, rhs <= sum(dual[e, e2] for e2 in rd.paths_edges[p]))
                end
            end
            
            # Enforce the timeout.
            if rd.timeout.value > 0 && Nanosecond(time_ns() - start) >= rd.timeout
                return RoutingModel(rd, m, UnitaryFlows, nothing)
            end
        end

        # Relate the main decision variables to the uncertainty sets.
        let dual_var = (rd.model_simplifications ? dual_beta : dual)
            # Splitting this constraint into multiple expressions can create
            # performance problems, as sum() is interpreted in a different way
            # within the macro (efficient code) compared to the outside code
            # (repeated calls to +=).
            @constraint(m, sum(dual_var[e, e2] * capacity(rd, e2) for e2 in edges(rd)) / capacity(rd, e) <= mu)
        end
    end

    if rd.model_simplifications
        return RoutingModel(rd, m, UnitaryFlows, rm.routing, mu=mu,
                            dual_alpha=dual_alpha, dual_beta=dual_beta,
                            constraints_uncertainty_set=dual_constraints,
                            constraints_convexity=rm.constraints_convexity)
    else
        return RoutingModel(rd, m, UnitaryFlows, rm.routing, mu=mu, dual=dual,
                            constraints_convexity=rm.constraints_convexity)
    end
end
