module Seleroute

using JuMP
using LightGraphs, MetaGraphs
using Compose, Colors, GraphPlot, Cairo, Fontconfig

using Printf, Statistics, LinearAlgebra, SparseArrays, DelimitedFiles, Random
import Base: summary, getindex
import LightGraphs: edges, vertices
# import JuMP: copy_model

const MOF = MOI.FileFormats

const AbstractSimpleGraph = LightGraphs.AbstractSimpleGraph

# Based on Optimal Oblivious Routing in Polynomial Time
# http://ttic.uchicago.edu/~harry/pdf/optimal_oblivious_journal.pdf
# Oblivious ratio: O(log^2 n log log n), n number of nodes
# http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.4.3883&rep=rep1&type=pdf: theorem 1, p. 9


inedges(g, v) = [Edge(iv, v) for iv in inneighbors(g, v)]
outedges(g, v) = [Edge(v, ov) for ov in outneighbors(g, v)]


# https://github.com/JuliaOpt/JuMP.jl/pull/1982
function write_LP(io, model::Model)
    lp_model = MOF.Model(format = MOF.FORMAT_LP)
    MOI.copy_to(lp_model, backend(model))
    MOI.write_to_file(lp_model, io)
end


# https://github.com/JuliaOpt/JuMP.jl/issues/2019
function Base.getindex(reference_map::ReferenceMap, container::Union{Containers.DenseAxisArray, Containers.SparseAxisArray})
    return getindex.(reference_map, container)
end


include("modeltype.jl")
include("data/parameters.jl")
include("data/model.jl")
include("data/solution.jl")
include("data/topology.jl")
include("export.jl")
include("basemodels/helpers.jl")
include("basemodels/knowncapacities.jl")
include("basemodels/knowncapacities_flow.jl")
include("basemodels/knowncapacities_path.jl")
include("basemodels/objectives.jl")
include("compute/compute.jl")
include("compute/metrics_evaluate.jl")
include("compute/certain/certain.jl")
include("compute/certain/mmf.jl")
include("compute/oblivious/iter.jl")
include("compute/oblivious/iter_pathcg.jl")
include("compute/oblivious/rr.jl")
include("compute/oblivious/rr_flow.jl")
include("compute/oblivious/rr_path.jl")
include("output/plots.jl")
include("output/reports.jl")
include("experiments/random.jl")

export EdgeWiseObjectiveFunction, AggregationObjectiveFunction, FormulationType,
       AlgorithmChoice, UncertaintyHandling, UncertainParameters, ModelType,
       Load, KleinrockLoad, FortzThorupLoad, AlphaFairness, MinimumTotal, MinimumMaximum, MinMaxFair,
       FlowFormulation, PathFormulation, Automatic, CuttingPlane, DualReformulation,
       NoUncertaintyHandling, StochasticUncertainty, RobustUncertainty,
       ObliviousUncertainty, NoUncertainty, UncertainDemand, UncertainCapacity,
       Topology, topology_to_graphs, remove_unreachable_nodes!,
       remove_unsatisfiable_demands!, Routing, routing_to_matrix, RoutingSolution,
       CertainRoutingSolution, flow_routing_to_path, path_routing_to_flow,
       RoutingData, edges, vertices, demands, n_nodes, n_edges, n_demands, n_paths,
       FlowScaling, UnitaryFlows, UnscaledFlows, RoutingModel, # copy_model,
       basic_routing_model_unitary, basic_routing_model_unscaled, total_flow_in_edge,
       capacity_constraints, mu_capacity_constraints,
       objective_edge_expression, solve_master_problem, solve_subproblem,
       compute_loads, compute_max_load, compute_routing,
       random_similar, random_experiment, random_experiments
       # Output?
end
