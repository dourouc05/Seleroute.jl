using Test, InteractiveUtils

using JuMP
using ECOS, SCS
using LightGraphs, MetaGraphs

using Seleroute

@testset "Seleroute.jl" begin
    @testset "Data" begin
        include("data_modeltype.jl")
        include("data_parameters.jl")
        include("data_solution.jl")
        include("data_topology.jl")
    end
    include("basemodels_knowncapacities.jl")
    @testset "Compute" begin
        include("compute_certain.jl")
        include("compute_oblivious_demand.jl")
    end
end
