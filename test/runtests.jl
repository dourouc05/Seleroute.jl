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
    include("compute.jl")
end
