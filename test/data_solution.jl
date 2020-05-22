@testset "Data: solution matrix to object" begin
    @testset "Edge flows" begin
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

            for i in eachindex(path_flows[d])
                @test path_flows[d][i] ≈ 0.5 atol=1.e-3
            end
            for i in eachindex(edge_flows[d])
                @test edge_flows[d][i] ≈ 0.5 atol=1.e-3
            end
        end
    end
end
