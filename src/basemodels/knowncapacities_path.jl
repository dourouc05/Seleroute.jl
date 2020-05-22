function basic_routing_model_unitary(m::Model, rd::RoutingData, ::PathFormulation)
    @variable(m, 0 <= routing[d in demands(rd), p in rd.demand_to_path_ids[d]] <= 1)
    @constraint(m, c_path[d in demands(rd)], sum(routing[d, p] for p in rd.demand_to_path_ids[d]) == 1)
    return RoutingModel(rd, m, UnitaryFlows, nothing, routing, constraints_convexity=c_path)
end

function basic_routing_model_unscaled(m::Model, rd::RoutingData, dm, ::PathFormulation)
    # Either dm is known (::Dict{Edge{Int}, Float64}) or is it a vector of variables (::DenseAxisArray).
    @variable(m, routing[d in demands(rd), p in rd.demand_to_path_ids[d]] >= 0)
    @constraint(m, c_path[d in demands(rd)], sum(routing[d, p] for p in rd.demand_to_path_ids[d]) == dm[d])
    return RoutingModel(rd, m, UnscaledFlows, nothing, routing, constraints_convexity=c_path)
end

function total_flow_in_edge(rm::RoutingModel, e::Edge, dm::Dict{Edge{Int}, Float64}, ::PathFormulation, ::Val{UnitaryFlows})
    flow = AffExpr(0.0)

    paths = _find_path_ids_with_edge(rm.data, e)
    for p in paths
        d = rm.data.path_id_to_demand[p]
        if d in keys(dm) && ! iszero(dm[d])
            add_to_expression!(flow, dm[d] * rm.routing[d, p])
        end
    end

    return flow
end

function total_flow_in_edge(rm::RoutingModel, e::Edge, ::PathFormulation, ::Val{UnscaledFlows})
    paths = _find_path_ids_with_edge(rm.data, e)
    if isempty(paths) # sum() over an empty array throws an error...
        return AffExpr(0.0)
    end
    return sum(rm.routing[rm.data.path_id_to_demand[p], p] for p in paths)
end
