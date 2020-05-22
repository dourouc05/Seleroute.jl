function _basic_routing_model_unitary_flow_conservation_flow_formulation(m::Model, rd::RoutingData, routing, f)
    # Write the flow-conservation constraints, the rhs being given by f. This generalises both unitary and unscaled models.
    cstrs_src_in = Dict{Edge, Any}()
    cstrs_src_out = Dict{Edge, Any}()
    cstrs_dst_in = Dict{Edge, Any}()
    cstrs_dst_out = Dict{Edge, Any}()
    cstrs_bal = Dict{Edge, Dict{Int, Any}}()

    for d in demands(rd)
        # TODO: only generate these constraints when there are edges considered (mostly for == 0: don't always generate them)
        cstrs_src_out[d] = @constraint(m, sum(routing[d, e] for e in outedges(rd.g, d.src)) == f(d))
        cstrs_src_in[d] = @constraint(m, sum(routing[d, e] for e in inedges(rd.g, d.src)) == 0)
        cstrs_dst_in[d] = @constraint(m, sum(routing[d, e] for e in inedges(rd.g, d.dst)) == f(d))
        cstrs_dst_out[d] = @constraint(m, sum(routing[d, e] for e in outedges(rd.g, d.dst)) == 0)

        set_name(cstrs_src_out[d], "c_conservation_source_out[$(d)]")
        set_name(cstrs_src_in[d], "c_conservation_source_in[$(d)]")
        set_name(cstrs_dst_in[d], "c_conservation_target_in[$(d)]")
        set_name(cstrs_dst_out[d], "c_conservation_target_out[$(d)]")

        cstrs_bal[d] = Dict{Int, Any}()
        for v in vertices(rd.g)
            if v == d.src || v == d.dst
                continue
            end

            cstrs_bal[d][v] = @constraint(m, sum(routing[d, e] for e in inedges(rd.g, v)) == sum(routing[d, e] for e in outedges(rd.g, v)))
            set_name(cstrs_bal[d][v], "c_conservation[$(d), $(v)]")
        end
    end

    return cstrs_src_in, cstrs_src_out, cstrs_dst_in, cstrs_dst_out, cstrs_bal
end

function basic_routing_model_unitary(m::Model, rd::RoutingData, ::FlowFormulation)
    @variable(m, 0 <= routing[d in demands(rd), e in edges(rd)] <= 1)

    cstrs_src_in, cstrs_src_out, cstrs_dst_in, cstrs_dst_out, cstrs_bal =
        _basic_routing_model_unitary_flow_conservation_flow_formulation(m, rd, routing, d -> 1)

    return RoutingModel(rd, m, UnitaryFlows, nothing, routing,
        constraints_source_in=cstrs_src_in, constraints_source_out=cstrs_src_out,
        constraints_target_in=cstrs_dst_in, constraints_target_out=cstrs_dst_out,
        constraints_balance=cstrs_bal)
end

function basic_routing_model_unscaled(m::Model, rd::RoutingData, dm::Dict{Edge, Float64}, ::FlowFormulation)
    # dm is either a traffic matrix or variables! The only guarantee is that it is indexable by demands.

    @variable(m, routing[d in demands(rd), e in edges(rd)] >= 0)

    cstrs_src_in, cstrs_src_out, cstrs_dst_in, cstrs_dst_out, cstrs_bal =
        _basic_routing_model_unitary_flow_conservation_flow_formulation(m, rd, routing, d -> dm[d])

    return RoutingModel(rd, m, UnscaledFlows, nothing, routing,
        constraints_source_in=cstrs_src_in, constraints_source_out=cstrs_src_out,
        constraints_target_in=cstrs_dst_in, constraints_target_out=cstrs_dst_out,
        constraints_balance=cstrs_bal)
end

function total_flow_in_edge(rm::RoutingModel, e::Edge, dm::Dict{Edge, Float64}, ::FlowFormulation, ::Val{UnitaryFlows})
    flow = AffExpr(0.0)
    for d in keys(dm)
        if ! iszero(dm[d])
            add_to_expression!(flow, dm[d] * rm.routing[d, e])
        end
    end
    return flow
end

function total_flow_in_edge(rm::RoutingModel, e::Edge, ::FlowFormulation, ::Val{UnscaledFlows})
    return sum(rm.routing[d, e] for d in demands(rm.data))
end
