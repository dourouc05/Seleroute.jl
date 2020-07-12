@testset "RoutingData" begin
    g = MetaDiGraph() # Directed graph.
    add_vertex!(g, :name, "A")
    add_vertex!(g, :name, "B")
    add_vertex!(g, :name, "C")
    add_vertex!(g, :name, "D")
    add_edge!(g, 1, 2, :capacity, 1.0)
    add_edge!(g, 1, 3, :capacity, 1.0)
    add_edge!(g, 2, 4, :capacity, 1.0)
    add_edge!(g, 3, 4, :capacity, 1.0)

    ug = MetaGraph() # Undirected graph (otherwise same as above).
    add_vertex!(ug, :name, "A")
    add_vertex!(ug, :name, "B")
    add_vertex!(ug, :name, "C")
    add_vertex!(ug, :name, "D")
    add_edge!(ug, 1, 2, :capacity, 1.0)
    add_edge!(ug, 1, 3, :capacity, 1.0)
    add_edge!(ug, 2, 4, :capacity, 1.0)
    add_edge!(ug, 3, 4, :capacity, 1.0)

    g_unreachable = MetaDiGraph()
    add_vertex!(g_unreachable, :name, "A")
    add_vertex!(g_unreachable, :name, "B")
    add_vertex!(g_unreachable, :name, "C")
    add_vertex!(g_unreachable, :name, "D")
    add_edge!(g_unreachable, 1, 2, :capacity, 1.0)
    add_edge!(g_unreachable, 2, 4, :capacity, 1.0)
    # Node C has no edge.

    k = SimpleDiGraph(4)
    add_edge!(k, 1, 4)

    uk = SimpleGraph(4)
    add_edge!(k, 1, 4)

    k_unreachable = SimpleDiGraph(4)
    add_edge!(k_unreachable, 1, 3)

    @testset "Helpers" begin
        @testset "remove_unreachable_nodes!" begin
            g_ = copy(g_unreachable)
            @test remove_unreachable_nodes!(g_) == 1
            @test ne(g_) == 2
            @test nv(g_) == 3
        end

        @testset "remove_unsatisfiable_demands!" begin
            g_ = copy(g_unreachable)
            k_ = copy(k_unreachable)
            @test remove_unsatisfiable_demands!(g_, k_) == 1
            @test ne(g_) == 2
            @test nv(g_) == 4
            @test ne(k_) == 0
        end
    end

    @testset "Parameters" begin
        @testset "Undirected graph" begin
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            @test_throws ErrorException RoutingData(copy(ug), copy(k), ECOS.Optimizer, mt)
            @test_throws ErrorException RoutingData(copy(g), copy(uk), ECOS.Optimizer, mt)
            @test_throws ErrorException RoutingData(copy(ug), copy(uk), ECOS.Optimizer, mt)
        end

        @testset "Nonexistent folder" begin
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            @test_throws ErrorException RoutingData(copy(g), copy(k), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
        end

        @testset "Unsatisfiable or unreachable parts" begin
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())

            tmp = AbstractString[]
            lm = (x) -> push!(tmp, x)
            RoutingData(copy(g_unreachable), copy(k), ECOS.Optimizer, mt, verbose=true, logmessage=lm)
            @test length(tmp) == 2
            @test "Removed 1 vertices because they were not reachable." in tmp
            @test "Removed 1 demands because they were not routable." in tmp
        end
    end

    @testset "RoutingData" begin
        set = [("Flow-based", FlowFormulation()), ("Path-based", PathFormulation())]

        @testset "$name" for (name, type) in set
            mt = ModelType(Load(), MinimumTotal(), type, false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            rd = RoutingData(copy(g), copy(k), ECOS.Optimizer, mt)

            @test ne(rd.g) == ne(g)
            @test nv(rd.g) == nv(g)
            @test ne(rd.k) == ne(k)
            @test nv(rd.k) == nv(k)
            @test rd.solver == ECOS.Optimizer
            @test rd.name == ""
            @test rd.time_precompute_ms > 0.0
            @test rd.model_type == mt
            @test rd.sub_model_type == mt
            @test length(rd.traffic_matrix) == 0 # TODO: is this right?

            @test length(edges(rd)) == 4
            @test length(vertices(rd)) == 4
            @test length(demands(rd)) == 1
            @test n_nodes(rd) == 4
            @test n_edges(rd) == 4
            @test n_demands(rd) == 1
            @test n_paths(rd) == (type == PathFormulation() ? 2 : 0)

            if type == PathFormulation()
                @test [Edge(1, 2), Edge(2, 4)] in rd.paths_edges
                @test [Edge(1, 3), Edge(3, 4)] in rd.paths_edges
                @test length(rd.demand_to_path_ids[Edge(1, 4)]) == 2
                @test rd.path_id_to_demand[1] == Edge(1, 4)
                @test rd.path_id_to_demand[2] == Edge(1, 4)
            else
                @test type == FlowFormulation()
                @test length(rd.paths_edges) == 0
                @test length(rd.demand_to_path_ids) == 0
                @test length(rd.path_id_to_demand) == 0
            end

            # TODO: what about the submodel type? Testing that it is essentially the same as the main object, also try to generate a different one.
        end

        @testset "Warning when generating too many paths" begin
            # Flow formulation.
            mt = ModelType(Load(), MinimumTotal(), FlowFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            @test_nowarn RoutingData(g, k, ECOS.Optimizer, mt, npaths=1)

            # Without column generation: warning.
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            wrnmsg = "The limit of paths to generate (1) has been reached; the result will not be " *
                  "guaranteed to be optimum. Switch to a column-generation formulation for exact results."
            @test_logs (:warn, wrnmsg) RoutingData(g, k, ECOS.Optimizer, mt, npaths=1)

            # With column generation: no warning.
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), true, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            @test_nowarn RoutingData(g, k, ECOS.Optimizer, mt, npaths=1)
        end

        @testset "find_path_ids_with_edge" begin
            # Flow formulation: throw errors.
            mt = ModelType(Load(), MinimumTotal(), FlowFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            rd = RoutingData(copy(g), copy(k), ECOS.Optimizer, mt)
            @test_throws ErrorException find_path_ids_with_edge(rd, Edge(1, 2))

            # Path formulation.
            mt = ModelType(Load(), MinimumTotal(), PathFormulation(), true, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
            rd = RoutingData(copy(g), copy(k), ECOS.Optimizer, mt)

            p_id = find_path_ids_with_edge(rd, Edge(1, 2))
            @test length(p_id) == 1
            @test Edge(1, 2) in rd.paths_edges[p_id[1]]

            p_id = find_path_ids_with_edge(rd, Edge(1, 5))
            @test length(p_id) == 0
        end
    end

    @testset "Objective functions: support_min/max" begin
        @test supports_min(Load())
        @test supports_max(Load())

        @test supports_min(KleinrockLoad())
        @test ! supports_max(KleinrockLoad())

        @test supports_min(FortzThorupLoad())
        @test ! supports_max(FortzThorupLoad())

        @test supports_min(AlphaFairness(0.0))
        @test supports_max(AlphaFairness(0.0))

        @test ! supports_min(AlphaFairness(0.5))
        @test supports_max(AlphaFairness(0.5))
    end
end
