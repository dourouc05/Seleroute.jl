module Seleroute

using JuMP
using Graphs, MetaGraphs
using Compose, Colors, GraphPlot, Cairo, Fontconfig
using Dates

using Printf, Statistics, LinearAlgebra, SparseArrays, DelimitedFiles, Random
import Base: summary, getindex
import Graphs: edges, vertices
# import JuMP: copy_model

const MOF = MOI.FileFormats

const AbstractSimpleGraph = Graphs.AbstractSimpleGraph

# TODO: for timeouts, it would be best if all the algorithms were implemented
# using solver callbacks, the mechanisms would be easier to implement and
# verify. Currently, it's really ad-hoc and I'm not really sure the returned
# values always make sense.


inedges(g, v) = [Edge(iv, v) for iv in inneighbors(g, v)]
outedges(g, v) = [Edge(v, ov) for ov in outneighbors(g, v)]


# https://github.com/JuliaOpt/JuMP.jl/pull/1982
function write_LP(io, model::Model)
    lp_model = MOF.Model(format = MOF.FORMAT_LP)
    MOI.copy_to(lp_model, backend(model))
    MOI.write_to_file(lp_model, io)
end

function write_MOF(io, model::Model)
    mof_model = MOF.Model(format = MOF.FORMAT_MOF)
    MOI.copy_to(mof_model, backend(model))
    MOI.write_to_file(mof_model, io)
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
include("compute/certain/certain_cg.jl")
include("compute/certain/mmf.jl")
include("compute/oblivious/iter.jl")
include("compute/oblivious/iter_pathcg.jl")
include("compute/oblivious/rr.jl")
include("compute/oblivious/rr_flow.jl")
include("compute/oblivious/rr_path.jl")
include("compute/oblivious/rr_pathcg.jl") # Must be included after iter_pathcg.jl due to shared helpers.
include("output/plots.jl")
include("output/reports.jl")
include("experiments/random.jl")

# Export all symbols. Code copied from JuMP.
const _EXCLUDE_SYMBOLS = [Symbol(@__MODULE__), :eval, :include]

for sym in names(@__MODULE__, all=true)
    sym_string = string(sym)
    if sym in _EXCLUDE_SYMBOLS || startswith(sym_string, "_")
        continue
    end
    if !(Base.isidentifier(sym) || (startswith(sym_string, "@") &&
         Base.isidentifier(sym_string[2:end])))
       continue
    end
    @eval export $sym
end

end
