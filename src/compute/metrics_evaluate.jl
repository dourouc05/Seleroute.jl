function compute_loads(rd::RoutingData, routing::Routing, demands::Dict{Edge{Int}, Float64})
    load = Dict{Edge{Int}, Float64}()
    for e in edges(rd)
        load[e] = 0.0
        for demand in keys(demands)
            if e in keys(routing.edge_flows[demand])
                load[e] += demands[demand] * routing.edge_flows[demand][e]
            end
        end
        load[e] /= get_prop(rd.g, e, :capacity)
    end
    return load
end

compute_max_load(routing::Routing, demands::Dict{Edge{Int}, Float64}) =
    maximum(values(compute_loads(routing.data, routing, demands)))

function compute_max_load(rd::RoutingData, demands::Dict{Edge{Int}, Float64})
    m = Model(rd.solver)
    set_silent(m)
    return compute_max_load(m, rd, demands)
end

function compute_max_load(m::Model, rd::RoutingData, demands::Dict{Edge{Int}, Float64})
    rm = basic_routing_model_unitary(m, rd, Val(rd.model_type.type))
    mu_capacity_constraints(rm, demands)
    @objective(m, Min, mu)
    optimize!(m)
    return objective_value(m)
end
