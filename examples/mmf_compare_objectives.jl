# In theory, for load-based MMF routing, using any monotone function of the load
# should give the same result. We check that fact. We proceed as follows:
# for each topology, for several objective functions (including the simple
# load), compute a routing; compare all those routings.

# For simplicity, we will focus on load functions that can be solved with
# a standard SOCP solver (like CPLEX, Gurobi, Mosek).

# Theoretical reference:
# https://ieeexplore.ieee.org/document/4738463 : Section 4.1.2.

include("topologies.jl")

using PrettyTables
using CPLEX, LightGraphs
opt = CPLEX.Optimizer

ftopologies = [t for t in topologies if ! (t.name in ["att", "abilene"])]

objs = [Load(), KleinrockLoad(), FortzThorupLoad(), AlphaFairness(0.5)]#, AlphaFairness(1.5), AlphaFairness(2.0)]
table = Matrix{Any}(undef, 2 * length(ftopologies), 2 + length(objs))
for (i, t) in enumerate(ftopologies)
    println("=> $(t.name)")
    g, k, dm = topology_to_graphs(t)

    loads = Any[]
    n_paths = Any[]

    for obj in objs
        println("  -> $(obj)")

        if typeof(obj) != AlphaFairness
            mt = ModelType(obj, MinMaxFair())
        else
            mt = ModelType(obj, MaxMinFair())
        end

        rd = RoutingData(g, k, opt, mt, name=t.name, traffic_matrix=dm)
        sol = compute_routing(rd)

        r = sol.routings[end]
        push!(loads, compute_max_load(r))
        push!(n_paths, length(r.paths))
    end

    table[2 * i - 1, :] = vcat([t.name, "Max load"], loads)
    table[2 * i, :] = vcat(["", "Nb. paths"], n_paths)
end

@ptconfclean
@ptconf tf = unicode_rounded
@pt :header = ["", "", objs...] table
