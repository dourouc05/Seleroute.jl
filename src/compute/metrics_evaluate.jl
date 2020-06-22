function compute_loads(routing::Routing, demands::Dict{Edge{Int}, Float64})
    rd = routing.data
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

compute_loads(routing::RoutingSolution) =
    compute_loads(routing.routings[end])

compute_loads(routing::Routing) =
    compute_loads(routing, routing.data.traffic_matrix)

compute_max_load(routing::Routing, demands::Dict{Edge{Int}, Float64}) =
    maximum(values(compute_loads(routing, demands)))

compute_max_load(routing::RoutingSolution) =
    compute_max_load(routing.routings[end])

compute_max_load(routing::Routing) =
    compute_max_load(routing, routing.data.traffic_matrix)

function compute_max_load(rd::RoutingData, demands::Dict{Edge{Int}, Float64})
    m = Model(rd.solver)
    set_silent(m)
    return compute_max_load(m, rd, demands)
end

function compute_max_load(m::Model, rd::RoutingData, demands::Dict{Edge{Int}, Float64})
    rm = basic_routing_model_unitary(m, rd, rd.model_type.type)
    mu_capacity_constraints(rm, demands)
    @objective(m, Min, rm.mu)
    optimize!(m)
    return objective_value(m)
end
