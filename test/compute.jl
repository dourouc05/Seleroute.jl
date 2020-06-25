@testset "Compute" begin
    # Required cones: SOCP (Kleinrock, α-fairness), EXP (α-fairness), POW (α-fairness).
    # ECOS only provides SOCP and EXP, use SCS for the rest (worse accuracy that ECOS).
    opt_ecos = ECOS.Optimizer
    opt_scs = SCS.Optimizer

    g = MetaDiGraph()
    add_vertex!(g, :name, "A")
    add_vertex!(g, :name, "B")
    add_vertex!(g, :name, "C")
    add_vertex!(g, :name, "D")
    add_edge!(g, 1, 2, :capacity, 2.0)
    add_edge!(g, 1, 3, :capacity, 2.0)
    add_edge!(g, 2, 4, :capacity, 2.0)
    add_edge!(g, 3, 4, :capacity, 2.0)
    add_edge!(g, 1, 4, :capacity, 3.0)
    paths = [[Edge(1, 4)], [Edge(1, 2), Edge(2, 4)], [Edge(1, 3), Edge(3, 4)]]

    k = SimpleDiGraph(4)
    add_edge!(k, 1, 4)
    d = Edge(1, 4)
    dm = Dict{Edge{Int}, Float64}(d => 4.0)

    # Include solver-specific tests.
    include("compute_certain.jl")
    @testset "Formulation type: $type" for type in [FlowFormulation(), PathFormulation()]
        @testset "Edge-wise objective: $edge_obj" for edge_obj in [
                Load(), KleinrockLoad(), FortzThorupLoad(),
                AlphaFairness(0.0, false), AlphaFairness(0.0, true),
                AlphaFairness(0.5, false), AlphaFairness(0.5, true),
                AlphaFairness(1.0),
                AlphaFairness(1.5, true), AlphaFairness(1.5, false),
                AlphaFairness(2.0, true), AlphaFairness(2.0, false)
            ]
            @testset "No uncertainty" begin
                opt = opt_ecos
                if typeof(edge_obj) == AlphaFairness && edge_obj.force_power_cone
                    opt = opt_scs
                end

                __testset_nouncertainty_minmax(edge_obj, type, opt, g, paths, k, d, dm)
                __testset_nouncertainty_mintot(edge_obj, type, opt, g, paths, k, d, dm)

                if typeof(edge_obj) != AlphaFairness || (! edge_obj.force_power_cone && edge_obj.α != 0.0)
                    # Power-cone models cannot be solved to optimality with ECOS or SCS,
                    # and they also sometimes have problems with linear models.
                    if ! (typeof(edge_obj) == AlphaFairness && edge_obj.α >= 1.0) # TODO: MMF doesn't work there: a bug somewhere?

                        __testset_nouncertainty_mmf(edge_obj, type, opt, g, paths, k, d, dm)
                    end
                end
            end
        end
    end

    include("compute_oblivious_demand.jl")
end
