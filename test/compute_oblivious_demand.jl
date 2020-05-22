@testset "Oblivious routing: uncertainty in the demand" begin
    opt = ECOS.Optimizer

    g = MetaDiGraph()
    add_vertex!(g, :name, "A")
    add_vertex!(g, :name, "B")
    add_vertex!(g, :name, "C")
    add_vertex!(g, :name, "D")
    add_vertex!(g, :name, "E")
    add_vertex!(g, :name, "F")
    add_edge!(g, 1, 3, :capacity, 2.0)
    add_edge!(g, 2, 3, :capacity, 2.0)
    add_edge!(g, 2, 6, :capacity, 2.0)
    add_edge!(g, 3, 4, :capacity, 20.0) # High capacity, shared.
    add_edge!(g, 4, 5, :capacity, 2.0)
    add_edge!(g, 1, 5, :capacity, 2.0) # Direct links.
    add_edge!(g, 4, 6, :capacity, 2.0)
    paths_d1 = [[Edge(1, 5)], [Edge(1, 3), Edge(3, 4), Edge(4, 5)]]
    paths_d2 = [[Edge(2, 6)], [Edge(2, 3), Edge(3, 4), Edge(4, 6)]]

    k = SimpleDiGraph(6)
    add_edge!(k, 1, 5)
    add_edge!(k, 2, 6)
    d1 = Edge(1, 5)
    d2 = Edge(2, 6)

    @testset "$type$(cg ? " with column generation" : ""))" for (type, cg) in [(FlowFormulation, false), (PathFormulation, false), (PathFormulation, true)]
        @testset "$algo" for algo in [CuttingPlane, DualReformulation]
            if cg && algo == DualReformulation
                # Not yet implemented.
                continue
            end

            mt = ModelType(Load, MinimumMaximum, type, cg, algo, ObliviousUncertainty, UncertainDemand)
            rd = RoutingData(g, k, opt, mt, npaths=(cg ? 1 : 5)) # Only two paths for each demand, force column generation to generate a new one.
            r = compute_routing(rd)

            @test r !== nothing
            @test r.data == rd
            @test r.n_iter > 0
            @test r.n_matrices >= 1
            if cg
                @test r.n_columns > 0
            else
                @test r.n_columns == 0
            end
            if algo == CuttingPlane
                @test r.n_cuts > 0
            else
                @test r.n_cuts == 0
            end
            @test r.time_precompute_ms > 0.0
            @test r.time_solve_ms > 0.0
            @test r.time_export_ms == 0.0
            @test length(r.objectives) >= 1
            @test length(r.matrices) >= 1
            @test length(r.routings) >= 1
            @test r.master_model !== nothing
        end
    end
end
