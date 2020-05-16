"""
Data structure for holding a routing in a path-based manner.
"""
struct Routing
    data::RoutingData
    path_flows::Dict{Edge, Dict{Int, Float64}} # Demand -> path -> flow.
    edge_flows::Dict{Edge, Dict{Edge, Float64}} # Demand -> edge -> flow.
    demands::Vector{Edge} # List of demand; the only goal of this vector is to fix
    # the order of the demands (i.e. have demand IDs).
    paths::Vector{Vector{Edge}} # A path is a list of edges. This vector stores
    # a mapping from integers (indices in the vector) to paths.

    function Routing(data::RoutingData, routing::AbstractMatrix{Float64})
        if data.model_type.type == FlowFormulation
            # If the model is not path-based, translate it.
            path_flows, edge_flows, paths = flow_routing_to_path(data, routing)
            return new(data, path_flows, edge_flows, collect(demands(data)), paths)
        else
            @assert data.model_type.type == PathFormulation
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
    # TODO: useful?
    # dm is an optimisation variable.
    return total_flow_in_edge(r, e, dm, Val(r.data.model_type.type))
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
    matrices::Vector{Dict{Edge, Float64}}
    routings::Vector{Routing}
    master_model::RoutingModel
end

function CertainRoutingSolution(data::RoutingData,
                                time_precompute_ms::Float64, time_solve_ms::Float64, time_export_ms::Float64,
                                mu::Float64, matrix::Dict{Edge, Float64}, routing::Routing, model::RoutingModel)
    return RoutingSolution(data, 0, 1, 0, 0, time_precompute_ms, time_solve_ms, time_export_ms,
                           Float64[mu], Dict{Edge, Float64}[matrix], Routing[routing], model)
end

function flow_routing_to_path(data::RoutingData, routing::AbstractMatrix{Float64}; demand=nothing)
    if data.model_type.type != FlowFormulation
        error("This function can only be used for flow-based formulations; it makes no sense for formulation type $(data.model_type)")
    end

    # For each demand, start at the source; follow edges to make up paths.
    # As soon as a path splits into multiple edges at some given vertex,
    # create several paths.
    path_flows = Dict{Edge, Dict{Int, Float64}}() # demand -> path index -> flow
    edge_flows = Dict{Edge, Dict{Edge, Float64}}() # demand -> edge -> flow
    paths = Vector{Edge}[] # list of paths (indexes match those of path_flows)

    for d in demands(data)
        if demand !== nothing && (!(d in keys(demand)) || iszero(demand[d]))
            continue
        end

        path_flows[d] = Dict{Int, Float64}()
        edge_flows[d] = Dict{Edge, Float64}()

        source = src(d)
        target = dst(d)

        # Store the list of paths for this demand. Map the *last* edge
        # of the currently built paths to a *list* of partial paths
        # (in case several paths diverge at some point, then end up
        # using the same edge).
        d_paths = Dict{Edge, Vector{Pair{Vector{Edge}, Float64}}}()
        #              ^^^^  ^^^^^^      ^^^^^^^^^^^^  ^^^^^^^
        #   one edge in the  paths       a path        the current weight
        #     path from the  being       starting at   of this path
        #     source to the  built       the source
        #       destination
        # The keys of d_paths are thus edges that should be further explore

        # Fill the list of paths with edges from the source.
        out_edges = collect(outedges(data.g, source))
        if length(out_edges) == 0
            error("No outgoing edge from the source $(source) in the input graph, is this really a routing?")
        end

        # Ensure that only edges with some flow are selected for further processing.
        for out_edge in out_edges
            weight = routing[d, out_edge]
            if iszero(weight)
                continue
            end
            edge_flows[d][out_edge] = weight

            d_paths[out_edge] = Pair{Vector{Edge}, Float64}[Edge[out_edge] => weight]
        end

        if length(d_paths) == 0
            error("No edge starting from the source detected in the solution; is this really a routing?")
        end

        # Iterate through d_paths until empty, filling path_flows/paths on the way when reaching the target.
        already_dealt_with = Set{Int}() # Nodes that have been met as sources of processed edges.
        while length(d_paths) >= 1
            # Take one edge at random to process it, remove it from d_paths.
            edge = first(keys(d_paths))
            partials = d_paths[edge] # Partial paths from the source and containing this edge.
            delete!(d_paths, edge)
            push!(already_dealt_with, src(edge))

            # Is this edge ending at the target? If so, found a complete path and its weight!
            if dst(edge) == target
                for partial in partials
                    if iszero(partial.second)
                        continue
                    end

                    push!(paths, partial.first)
                    path_flows[d][length(paths)] = partial.second
                end
                continue
            end

            # The exploration has not yet met the target: consider all outgoing
            # edges from the destination of the current edge.
            out_edges = collect(outedges(data.g, dst(edge)))
            if length(out_edges) == 0
                error("No outgoing edge from $(dst(edge)), how can a path go through it?")
            end

            for out_edge in out_edges
                # Compute the total weight going through this edge.
                weight = routing[d, out_edge]
                if iszero(weight)
                    continue
                end
                if ! (out_edge in keys(edge_flows[d]))
                    edge_flows[d][out_edge] = 0.0
                end
                edge_flows[d][out_edge] += weight

                # Add this new edge to the list of edges to process in a next iteration.
                if ! (out_edge in keys(d_paths))
                    d_paths[out_edge] = Pair{Vector{Edge}, Float64}[]
                end

                weight_normalisation = sum(partial.second for partial in partials)
                for partial in partials
                    new_partial_path = vcat(partial.first, out_edge)
                    new_partial_weight = weight * partial.second / weight_normalisation
                    push!(d_paths[out_edge], new_partial_path => new_partial_weight)
                end
            end
        end
    end

    return path_flows, edge_flows, paths
end

function path_routing_to_flow(data::RoutingData, routing::AbstractMatrix{Float64})
    if data.model_type.type != PathFormulation
        error("This function can only be used for path-based formulations; it makes no sense for formulation type $(data.model_type)")
    end

    # Sum over all paths that cross a given edge to get the total weight of each edge, demand per demand.
    path_flows = Dict{Edge, Dict{Int, Float64}}()

    for k in eachindex(routing)
        demand = k[1]
        path = k[2]

        if ! (demand in keys(path_flows))
            path_flows[demand] = Dict{Edge, Float64}()
        end

        if ! iszero(routing[k])
            path_flows[demand][path] = routing[k]
        end
    end

    # Sum the flows over the demands.
    edge_flows = Dict{Edge, Dict{Edge, Float64}}()
    for demand in demands(data)
        edge_flows[demand] = Dict{Edge, Float64}()
        for edge in edges(data)
            total_weight = zero(Float64)
            for p_id in _find_path_ids_with_edge(data, edge)
                if data.path_id_to_demand[p_id] == demand
                    total_weight += routing[demand, p_id]
                end
            end

            if ! iszero(total_weight)
                edge_flows[demand][edge] = total_weight
            end
        end
    end

    return path_flows, edge_flows
end
