# Compare all four implementations of demand-oblivious routing; they should
# give (approximately) the same result.

include("topologies.jl")

using PrettyTables, Graphs, DelimitedFiles
using JuMP, CPLEX, Gurobi, Mosek, MosekTools

opt = CPLEX.Optimizer
# opt = Gurobi.Optimizer
# opt = optimizer_with_attributes(Gurobi.Optimizer, "NonConvex" => 2)
# opt = Mosek.Optimizer

algos = [(CuttingPlane(), FlowFormulation(), false),
         (CuttingPlane(), PathFormulation(), false),
         (CuttingPlane(), PathFormulation(), true),
         (DualReformulation(), FlowFormulation(), false),
         (DualReformulation(), PathFormulation(), false),
         (DualReformulation(), PathFormulation(), true)]
headers = ["Cutting plane, flow formulation", "Cutting plane, path formulation", "Cutting plane, column generation",
           "Dual reformulation, flow formulation", "Dual reformulation, path formulation", "Dual reformulation, column generation"]

ftopologies = [t for t in topologies if ! (t.name in ["att", "abilene"])]
table_ratio = Matrix{Any}(undef, length(ftopologies), length(algos))
table_time = Matrix{Any}(undef, length(ftopologies), length(algos))

for (i, t) in enumerate(ftopologies)
    println("=> $(t.name)")
    g, k, dm = topology_to_graphs(t)

    for (j, (impl_algo, impl_form, impl_cg)) in enumerate(algos)
        println("  -> $(impl_algo), $(impl_form), column generation: $(impl_cg)")

        mt = ModelType(Load(), MinimumMaximum(), impl_form, impl_cg, impl_algo, ObliviousUncertainty(), UncertainDemand())
        rd = RoutingData(g, k, opt, mt, name=t.name, logmessage=(x)->nothing)
        sol = compute_routing(rd)

        table_ratio[i, j] = sol.objectives[sol.n_iter]
        table_time[i, j] = sol.total_time_ms
    end
end

@ptconfclean
@ptconf tf = unicode_rounded
println("\n\n  Ratios\n==========")
@pt :header = headers table_ratio
println("\n\n  Total times [ms]\n====================")
@pt :header = headers table_time
