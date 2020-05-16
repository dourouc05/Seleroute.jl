"""
Generates a few plots that summarise a solution.
"""
function plot(rd::RoutingData, flows;
              basename="$(@__DIR__)",
              sourceNode=colorant"orange", destinationNode=colorant"green", intermediateNode=colorant"steelblue",
              usedEdge=colorant"red", unusedEdge=colorant"lightgray")
    # Ensures the graph is no more directed: otherwise, risk of plotting y -> x on top of x -> y,
    # which hides the edge x -> y.
    g = SimpleGraph(SimpleDiGraph(rd.g))

    # Make a matrix out of the flows (they have a sparse representation).
    routing_matrix = zeros(n_demands(rd), n_paths(rd))
    for k in eachindex(flows)
        routing_matrix[k[1], k[2]] = flows[k...]
    end

    # If the locations of the nodes have not yet been computed, do it now.
    if rd.locs_x == nothing || rd.locs_y == nothing
        rd.locs_x, rd.locs_y = rd.locs_f(rd.g)
    end

    node_names = [get_prop(rd.g, n, :name) for n in vertices(rd.g)]

    # First, plot the complete graph, with just topology information.
    draw(PNG("$(basename).png"), gplot(g, rd.locs_x, rd.locs_y, nodelabel=node_names))

    # Then, plot the total load of each edge.
    paths = sum(routing_matrix[d, :] for d in 1:n_demands(rd))
    ps = findall(paths .> 0)
    edge_colours = [any(in(e, rd.paths_edges[p]) for p in ps) ? usedEdge : unusedEdge for e in edges(g)]
    edge_widths = [any(in(e, rd.paths_edges[p]) for p in ps) ?
                        2. * sum(paths[p] * in(e, rd.paths_edges[p]) for p in ps) :
                        1.0
                        for e in edges(g)]

    draw(
        PNG("$(basename)_load.png"),
        gplot(g, rd.locs_x, rd.locs_y, nodelabel=node_names, edgestrokec=edge_colours, edgelinewidth=edge_widths)
    )

    # Finally, plot each demand separately with the load it creates on each edge.
    vertex_to_colour(i, k_d) = i == src(k_d) ? sourceNode : (i == dst(k_d) ? destinationNode : intermediateNode)
    for d in 1:n_demands(rd)
        k_d = rd.demand_id_to_demand[d]
        node_colours = [vertex_to_colour(i, k_d) for i in vertices(g)]

        d_paths = routing_matrix[d, :]
        ps = findall(d_paths .> 0)
        edge_colours = [any(in(e, rd.paths_edges[p]) for p in ps) ? usedEdge : unusedEdge for e in edges(g)]
        edge_widths = [any(in(e, rd.paths_edges[p]) for p in ps) ?
                            2. * sum(d_paths[p] * in(e, rd.paths_edges[p]) for p in ps) :
                            1.0
                            for e in edges(g)]

        draw(
            PNG("$(basename)_$d.png"),
            gplot(g, rd.locs_x, rd.locs_y, nodelabel=node_names, nodefillc=node_colours,
                edgestrokec=edge_colours, edgelinewidth=edge_widths)
        )
    end
end
