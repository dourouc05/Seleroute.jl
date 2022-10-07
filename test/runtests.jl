using Test, InteractiveUtils

using JuMP
using ECOS, SCS
using Graphs, MetaGraphs

using Seleroute

struct TestEdgeWise <: EdgeWiseObjectiveFunction end

@testset "Seleroute.jl" begin
    @testset "Data" begin
        include("data_modeltype.jl")
        include("data_parameters.jl")
        include("data_solution.jl")
        include("data_topology.jl")
    end
    @testset "Base models" begin
        include("basemodels_knowncapacities.jl")
        include("basemodels_objectives.jl")
    end
    @testset "Compute" begin
        include("compute_certain.jl")
        include("compute_oblivious_demand.jl")
    end
end
