# Hold a topology in a format that should be close to what is available from
# network operators.
struct Topology
    name::AbstractString # Name (optional).
    nodes::Matrix{Any} # n×2 matrix: first column is a numerical index (Int),
    # second column si a node name.
    edges::Matrix{Float64} # m×3 matrix: first column is the index of the edge
    # source, second column is the index of the edge destination, third column
    # is the capacity of this link (no unit mandated; the user is responsible
    # for consistency).
    traffic::Matrix{Float64} # m×3 matrix: first column is the index of the
    # source node for this demand, second column is the index of the destination
    # node for this demand, third column is the actual demand (no unit mandated;
    # the user is responsible for consistency).
end

Topology(nodes::Matrix{Any}, edges::Matrix{Float64}, traffic::Matrix{Float64}) =
    Topology("", nodes, edges, traffic)

function topology_to_graphs(t::Topology; make_network_undirected::Bool=true)
    # make_network_undirected: whether edges should be duplicated to make the
    # network effectively undirected.

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
    dm = Dict{Edge{Int}, Float64}()
    sizehint!(dm, size(t.nodes, 1))

    for i in 1:size(t.nodes, 1)
        add_vertex!(k, :name, t.nodes[i, 2])
    end
    for i in 1:size(t.traffic, 1)
        ds = Int(t.traffic[i, 1])
        dt = Int(t.traffic[i, 2])
        add_edge!(k, ds, dt, :demand, t.traffic[i, 3]) # TODO: deprecate :demand here? So that all metadata from the graph is ignored. This should ease implementation of stochastic uncertainty.
        dm[Edge(ds, dt)] = t.traffic[i, 3]
    end

    return g, k, dm
end
