using ECOS
using Test

using JuMP
using LightGraphs, MetaGraphs

using Seleroute

@testset "Routings" begin
    include("topology.jl")
    include("data_parameters.jl")
    include("basemodels_knowncapacities.jl")
    include("compute.jl")
end
