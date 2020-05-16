struct Topology
    nodes::Array{Any, 2}
    edges::Array{Float64, 2}
    traffic::Array{Float64, 2}
end

function topology_to_graphs(t::Topology; make_network_undirected::Bool=true)
    # make_network_undirected: whether edges should be duplicated to make the network effectively undirected.

    # Topology.
    g = MetaDiGraph()
    for i in 1:size(t.nodes, 1)
        add_vertex!(g, :name, t.nodes[i, 2])
    end
    for i in 1:size(t.edges, 1)
        add_edge!(g, Int(t.edges[i, 1]), Int(t.edges[i, 2]), :capacity, t.edges[i, 3])
        if make_network_undirected
            add_edge!(g, Int(t.edges[i, 2]), Int(t.edges[i, 1]), :capacity, t.edges[i, 3])
        end
    end

    # Demands.
    k = MetaDiGraph()
    for i in 1:size(t.nodes, 1)
        add_vertex!(k, :name, t.nodes[i, 2])
    end
    for i in 1:size(t.traffic, 1)
        add_edge!(k, Int(t.traffic[i, 1]), Int(t.traffic[i, 2]), :demand, t.traffic[i, 3])
    end

    return g, k
end
