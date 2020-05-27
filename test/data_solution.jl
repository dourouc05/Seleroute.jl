@testset "Data: solution objects" begin
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
    paths_d1_idx = [1, 2]
    paths_d2_idx = [3, 4]

    k = SimpleDiGraph(6)
    add_edge!(k, 1, 5)
    add_edge!(k, 2, 6)
    d1 = Edge(1, 5)
    d2 = Edge(2, 6)

    @testset "Flow solution" begin
        # Taken from an actual solution, with some numerical difficulties removed.
        # This is already a poor-quality solution, expect deviations like 0.001.
        sm = JuMP.Containers.DenseAxisArray{Float64}(undef, edges(k), edges(g))
        sm[d1, Edge(1, 3)] = 0.5003495146197712
        sm[d1, Edge(2, 3)] = 1.1217083702874072e-11
        sm[d1, Edge(2, 6)] = 1.1217083702874072e-11
        sm[d1, Edge(3, 4)] = 0.5003495146197712
        sm[d1, Edge(4, 5)] = 0.5003495146197712
        sm[d1, Edge(1, 5)] = 0.49965048538022855
        sm[d1, Edge(4, 6)] = 1.1217083702874072e-11
        sm[d2, Edge(1, 3)] = -1.1163257993389714e-11
        sm[d2, Edge(2, 3)] = 0.5003495146197711
        sm[d2, Edge(2, 6)] = 0.49965048538022877
        sm[d2, Edge(3, 4)] = 0.5003495146197711
        sm[d2, Edge(4, 5)] = -1.1163257993389714e-11
        sm[d2, Edge(1, 5)] = -1.1163257993389714e-11
        sm[d2, Edge(4, 6)] = 0.5003495146197711

        mt = ModelType(Load(), MinimumMaximum(), FlowFormulation(), false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, nothing, mt)

        @testset "flow_routing_to_path" begin
            @test_throws ErrorException path_routing_to_flow(rd, sm)
            path_flows, edge_flows, paths = flow_routing_to_path(rd, sm)

            @test length(path_flows) == 2
            @test length(edge_flows) == 2
            @test length(paths) == 4

            @test [Edge(1, 5)] in paths
            @test [Edge(2, 6)] in paths
            @test [Edge(1, 3), Edge(3, 4), Edge(4, 5)] in paths
            @test [Edge(2, 3), Edge(3, 4), Edge(4, 6)] in paths

            for d in [d1, d2]
                @test length(path_flows[d]) == 2
                @test length(edge_flows[d]) == 4
            end

            # Paths with almost zero flow (below numerical accuracy) should not exist.
            for i in paths_d1_idx
                @test path_flows[d1][i] ≈ 0.5 atol=1.e-3
                @test_throws KeyError path_flows[d2][i]
            end
            for i in paths_d2_idx
                @test_throws KeyError path_flows[d1][i]
                @test path_flows[d2][i] ≈ 0.5 atol=1.e-3
            end
        end

        @testset "routing_to_matrix" begin
            r = Routing(rd, sm)

            mat = routing_to_matrix(r)
            @test mat[1, 1] ≈ 0.5 atol=1.e-3
            @test mat[1, 2] ≈ 0.5 atol=1.e-3
            @test mat[1, 3] ≈ 0.0 atol=1.e-3
            @test mat[1, 4] ≈ 0.0 atol=1.e-3
            @test mat[2, 1] ≈ 0.0 atol=1.e-3
            @test mat[2, 2] ≈ 0.0 atol=1.e-3
            @test mat[2, 3] ≈ 0.5 atol=1.e-3
            @test mat[2, 4] ≈ 0.5 atol=1.e-3
        end
    end

    @testset "Path solution" begin
        # Very similar solution to the flow one.
        sm = JuMP.Containers.SparseAxisArray{Float64, 2, Tuple{Edge{Int}, Int}}(Dict())
        for p in paths_d1_idx
            sm[d1, p] = 0.5003495146197712
            sm[d2, p] = 1.1217083702874072e-11
        end
        for p in paths_d2_idx
            sm[d1, p] = -1.1163257993389714e-11
            sm[d2, p] = 0.49965048538022855
        end

        mt = ModelType(Load(), MinimumMaximum(), PathFormulation(), false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, nothing, mt)

        @testset "path_routing_to_flow" begin
            @test_throws ErrorException flow_routing_to_path(rd, sm)
            path_flows, edge_flows = path_routing_to_flow(rd, sm)

            @test length(path_flows) == 2
            @test length(edge_flows) == 2

            for d in [d1, d2]
                @test length(path_flows[d]) == 2
                @test length(edge_flows[d]) == 4
            end

            # Paths with almost zero flow (below numerical accuracy) should not exist.
            for i in paths_d1_idx
                @test path_flows[d1][i] ≈ 0.5 atol=1.e-3
                @test_throws KeyError path_flows[d2][i]
            end
            for i in paths_d2_idx
                @test_throws KeyError path_flows[d1][i]
                @test path_flows[d2][i] ≈ 0.5 atol=1.e-3
            end
        end

        @testset "routing_to_matrix" begin
            r = Routing(rd, sm)

            mat = routing_to_matrix(r)
            @test mat[1, 1] ≈ 0.5 atol=1.e-3
            @test mat[1, 2] ≈ 0.5 atol=1.e-3
            @test mat[1, 3] ≈ 0.0 atol=1.e-3
            @test mat[1, 4] ≈ 0.0 atol=1.e-3
            @test mat[2, 1] ≈ 0.0 atol=1.e-3
            @test mat[2, 2] ≈ 0.0 atol=1.e-3
            @test mat[2, 3] ≈ 0.5 atol=1.e-3
            @test mat[2, 4] ≈ 0.5 atol=1.e-3
        end
    end
end
