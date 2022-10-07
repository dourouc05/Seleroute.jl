# In theory, for load-based MMF routing, using any monotone function of the load
# should give the same result. We check that fact. We proceed as follows:
# for each topology, for several objective functions (including the simple
# load), compute a routing; compare all those routings.

# For simplicity, we will focus on load functions that can be solved with
# a standard SOCP solver (like CPLEX, Gurobi, Mosek).

# Theoretical reference:
# https://ieeexplore.ieee.org/document/4738463 : Section 4.1.2.

include("topologies.jl")

using PrettyTables, Graphs, DelimitedFiles
using JuMP, CPLEX, Gurobi, Mosek, MosekTools

# opt = CPLEX.Optimizer
opt = optimizer_with_attributes(Gurobi.Optimizer, "NonConvex" => 2)
# opt = Mosek.Optimizer

ftopologies = [t for t in topologies if ! (t.name in ["att", "abilene"])]
ε = 1.0e-8

objs = [Load(), KleinrockLoad(true), FortzThorupLoad()]#, AlphaFairness(0.5)]#, AlphaFairness(1.0)]#, AlphaFairness(1.5), AlphaFairness(2.0)]
table = Matrix{Any}(undef, 2 * length(ftopologies), 2 + length(objs))
csv_cols = ["Instance", "Number of edges", "Number of nodes", "Number of demands",
            "Objective function for MMF", "MMF iterations", "Time to create the master problem [ms]",
            "Computation time (per iteration) [ms]", "Maximum iteration time [ms]", "Total iteration time [ms]",
            "Total time [ms]", "Maximum load of the obtained routing", "MMF status code"]
csv = Matrix{String}(undef, 1 + length(ftopologies) * length(objs), length(csv_cols))
csv[1, :] = csv_cols
csv_row = 2

for (i, t) in enumerate(ftopologies)
    println("=> $(t.name)")
    g, k, dm = topology_to_graphs(t)

    loads = Any[]
    n_paths = Any[]

    for obj in objs
        println("  -> $(obj)")

        if typeof(obj) != AlphaFairness
            mt = ModelType(obj, MinMaxFair(ε))
        else
            mt = ModelType(obj, MaxMinFair(ε))
        end

        rd = RoutingData(g, k, opt, mt, name=t.name, traffic_matrix=dm)
        sol = compute_routing(rd)

        r = sol.routings[sol.n_iter]
        max_load = compute_max_load(r)
        push!(loads, max_load)
        push!(n_paths, length(r.paths))

        global csv_row
        csv[csv_row, :] = String[t.name, string(ne(g)), string(nv(g)), string(ne(k)),
                                 string(obj), string(sol.n_iter), string(sol.time_create_master_model_ms),
                                 string(sol.time_solve_ms), string(maximum(values(sol.time_solve_ms))), string(sol.total_time_solve_ms),
                                 string(sol.total_time_ms), string(max_load), string(sol.result)]
        csv_row += 1
    end

    table[2 * i - 1, :] = vcat([t.name, "Max load"], loads)
    table[2 * i, :] = vcat(["", "Nb. paths"], n_paths)
end

@ptconfclean
@ptconf tf = unicode_rounded
@pt :header = ["", "", objs...] table

# println(csv)
writedlm("$(@__DIR__)/mmf_compare_objectives.csv", csv, ',')
