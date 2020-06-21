# In theory, for load-based MMF routing, using any monotone function of the load
# should give the same result. We check that fact. We proceed as follows:
# for each topology, for several objective functions (including the simple
# load), compute a routing; compare all those routings.

# For simplicity, we will focus on load functions that can be solved with
# a standard SOCP solver (like CPLEX, Gurobi, Mosek).

# Theoretical reference:
# https://ieeexplore.ieee.org/document/4738463 : Section 4.1.2.

include("topologies.jl")

using CPLEX, LightGraphs
opt = CPLEX.Optimizer

objs = [Load(), KleinrockLoad(), FortzThorupLoad(), AlphaFairness(0.5), AlphaFairness(1.5), AlphaFairness(2.0)]
for t in topologies
    if t.name in ["att", "abilene"]
        continue
    end

    println("=> $(t.name)")
    g, k, dm = topology_to_graphs(t)

    println(g)
    println(collect(edges(g)))
    println(k)
    println(collect(edges(k)))
    println(dm)

    for obj in objs
        println("  -> $(obj)")
        mt = ModelType(obj, MinMaxFair())
        rd = RoutingData(g, k, opt, mt, name=t.name, traffic_matrix=dm)
        sol = compute_routing(rd)
    end
end
