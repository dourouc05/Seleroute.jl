"""
Data structure for holding a routing in a path-based manner.
"""
struct Routing
    data::RoutingData
    path_flows::Dict{Edge{Int}, Dict{Int, Float64}} # Demand -> path -> flow.
    edge_flows::Dict{Edge{Int}, Dict{Edge{Int}, Float64}} # Demand -> edge -> flow.
    demands::Vector{Edge} # List of demand; the only goal of this vector is to fix
    # the order of the demands (i.e. have demand IDs).
    paths::Vector{Vector{Edge}} # A path is a list of edges. This vector stores
    # a mapping from integers (indices in the vector) to paths.

    function Routing(data::RoutingData, routing::AbstractMatrix{Float64})
        if data.model_type.type == FlowFormulation()
            # If the model is not path-based, translate it.
            path_flows, edge_flows, paths = flow_routing_to_path(data, routing)
            return new(data, path_flows, edge_flows, collect(demands(data)), paths)
        else
            @assert data.model_type.type == PathFormulation()
            # Otherwise, much simpler transformation: the solution is already written in terms of paths.
            path_flows, edge_flows = path_routing_to_flow(data, routing)
            return new(data, path_flows, edge_flows, collect(demands(data)), data.paths_edges)
        end
    end
end

function routing_to_matrix(r::Routing)
    routing_matrix = zeros(length(r.demands), length(r.paths))
    for demand_id in 1:length(r.demands)
        demand = r.demands[demand_id]
        for (path, weight) in r.path_flows[demand]
            routing_matrix[demand_id, path] = weight
        end
    end
    return routing_matrix
end

function total_flow_in_edge(r::Routing, e::Edge, dm)
    # dm is an optimisation variable, as the routing is fixed.
    return total_flow_in_edge(r, e, dm, r.data.model_type.type)
end

function total_flow_in_edge(r::Routing, e::Edge, dm, ::FormulationType; ε::Float64=1.e-3)
    # dm is an optimisation variable, as the routing is fixed.
    # Works for both flow-based and path-based formulations.
    # Cannot use sum(), as the collection is sometimes empty.
    flow = 0.0
    for d in demands(r.data)
        if e in keys(r.edge_flows[d]) && abs(r.edge_flows[d][e]) > ε
            flow += r.edge_flows[d][e] * dm[d]
        end
    end
    return flow
end

"""
Data structure holding results of the computations for a routing.
All information from intermediate iterations are kept within this object.

  * `data`: a pointer to the `RoutingData` object that was used for the computations
  * `n_iter`: the number of iterations
  * `n_matrices`: the number of matrices added into the formulation
  * `n_cuts`: the number of added constraints.

The following vectors contain one entry per iteration (i.e. entry 1 is the first iteration).

  * `objectives`: the evolution of the oblivious rations over the iterations.
  * `matrices`: the demand matrices generated during the execution. There is no such matrix at the last iteration.
  * `routings`: the various routings generated during the execution.
"""
struct RoutingSolution
    data::RoutingData
    n_iter::Int
    n_matrices::Int
    n_cuts::Int
    n_columns::Int

    time_precompute_ms::Float64
    time_solve_ms::Float64
    time_export_ms::Float64

    objectives::Vector{Float64}
    matrices::Vector{Dict{Edge{Int}, Float64}}
    routings::Vector{Routing}
    master_model::RoutingModel
end

function CertainRoutingSolution(data::RoutingData,
                                time_precompute_ms::Float64, time_solve_ms::Float64, time_export_ms::Float64,
                                mu::Float64, matrix::Dict{Edge{Int}, Float64}, routing::Routing, model::RoutingModel)
    return RoutingSolution(data, 0, 1, 0, 0, time_precompute_ms, time_solve_ms, time_export_ms,
                           Float64[mu], Dict{Edge{Int}, Float64}[matrix], Routing[routing], model)
end

function flow_routing_to_path(data::RoutingData, routing::AbstractMatrix{Float64}; demand=nothing, ε::Float64=1.e-5)
    if data.model_type.type != FlowFormulation()
        error("This function can only be used for flow-based formulations; it makes no sense for formulation type $(data.model_type)")
    end

    path_flows = Dict{Edge{Int}, Dict{Int, Float64}}() # demand -> path index -> flow
    edge_flows = Dict{Edge{Int}, Dict{Edge{Int}, Float64}}() # demand -> edge -> flow
    paths = Vector{Edge}[] # list of paths (indexes match those of path_flows)

    for d in demands(data)
        # Prepare to store the results for this demand.
        path_flows[d] = Dict{Int, Float64}()
        edge_flows[d] = Dict{Edge{Int}, Float64}()

        # Copy the flows into a new graph (with only those edges).
        # Don't use a weighted graph: removing edges does not always work
        # as expected.
        ug = MetaGraph(nv(graph(data)))
        for e in edges(data)
            if has_edge(ug, src(e), dst(e))
                continue
            end

            flow_a = try; routing[d, e]; catch; 0.0; end
            flow_b = try; routing[d, reverse(e)]; catch; 0.0; end
            flow = abs(flow_a - flow_b)
            if flow > ε
                add_edge!(ug, src(e), dst(e), :weight, flow)
            end
        end

        # Compute paths and remove the corresponding flow as long as there is
        # something in there.
        weight_e(e) = get_prop(ug, e, :weight)
        while ne(ug) > 0
            # Get a path arbitrarily (easiest to compute: shortest one)
            sp = desopo_pape_shortest_paths(ug, src(d))
            path_vertices = enumerate_paths(sp, dst(d))
            path = map(Edge, zip(path_vertices[1:end-1], path_vertices[2:end]))
            if length(path) == 0
                break
            end
            path_weight = minimum(weight_e(e) for e in path)

            # Update the data structures with this new path.
            push!(paths, path)
            path_idx = length(paths)
            path_flows[d][path_idx] = path_weight
            for e in path # TODO: DataStructures.jl's DefaultDict? Potential problems when returning this instead of a Dict?
                if ! haskey(path_flows[d], e)
                    edge_flows[d][e] = 0.0
                end
                edge_flows[d][e] += path_weight
            end

            # Remove the path from the graph.
            for e in path
                new_weight = weight_e(e) - path_weight
                if new_weight > ε
                    set_prop!(ug, e, :weight, new_weight)
                else
                    rem_edge!(ug, e)
                end
            end
        end
    end

    return path_flows, edge_flows, paths
end

function path_routing_to_flow(data::RoutingData, routing::AbstractMatrix{Float64}; ε::Float64=1.e-5)
    if data.model_type.type != PathFormulation()
        error("This function can only be used for path-based formulations; it makes no sense for formulation type $(data.model_type)")
    end

    # Sum over all paths that cross a given edge to get the total weight of each edge, demand per demand.
    path_flows = Dict{Edge{Int}, Dict{Int, Float64}}()

    for k in eachindex(routing)
        demand = k[1]
        path = k[2]

        if ! (demand in keys(path_flows))
            path_flows[demand] = Dict{Int, Float64}()
        end

        if abs(routing[k]) > ε
            path_flows[demand][path] = routing[k]
        end
    end

    # Sum the flows over the demands.
    edge_flows = Dict{Edge{Int}, Dict{Edge{Int}, Float64}}()
    for demand in demands(data)
        edge_flows[demand] = Dict{Edge{Int}, Float64}()
        for edge in edges(data)
            total_weight = zero(Float64)
            for p_id in _find_path_ids_with_edge(data, edge)
                if data.path_id_to_demand[p_id] == demand
                    total_weight += routing[demand, p_id]
                end
            end

            if abs(total_weight) > ε
                edge_flows[demand][edge] = total_weight
            end
        end
    end

    return path_flows, edge_flows
end
