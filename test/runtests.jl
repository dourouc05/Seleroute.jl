using Test, InteractiveUtils

using JuMP
using ECOS, SCS
using LightGraphs, MetaGraphs

using Seleroute

@testset "Routings" begin
    include("topology.jl")
    include("data_parameters.jl")
    include("data_solution.jl")
    include("basemodels_knowncapacities.jl")
    @testset "Compute" begin
        include("compute_certain.jl")
        include("compute_oblivious_demand.jl")
    end
end
