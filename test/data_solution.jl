@testset "Solution objects" begin
    @testset "Graph 1" begin
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

        k = SimpleDiGraph(6)
        add_edge!(k, 1, 5)
        add_edge!(k, 2, 6)
        d1 = Edge(1, 5)
        d2 = Edge(2, 6)

        paths_d1 = [[Edge(1, 5)], [Edge(1, 3), Edge(3, 4), Edge(4, 5)]]
        paths_d2 = [[Edge(2, 6)], [Edge(2, 3), Edge(3, 4), Edge(4, 6)]]
        paths_d1_idx = [1, 2]
        paths_d2_idx = [3, 4]

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

    @testset "Graph 2" begin # Same graph (and solution) as in compute.jl, for α-fairness.
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

        k = SimpleDiGraph(4)
        add_edge!(k, 1, 4)
        d = Edge(1, 4)
        dm = Dict{Edge{Int}, Float64}(d => 4.0)

        paths_d = [[Edge(1, 2), Edge(2, 4)], [Edge(1, 3), Edge(3, 4)], [Edge(1, 4)]]

        @testset "Flow solution" begin
            # Taken from an actual solution, with some numerical difficulties removed.
            # This is already a poor-quality solution, expect deviations like 0.001.
            sm = JuMP.Containers.DenseAxisArray{Float64}(undef, edges(k), edges(g))
            sm[d, Edge(1, 4)] = 0.30413786305612406
            sm[d, Edge(1, 2)] = 0.347931068471938
            sm[d, Edge(1, 3)] = 0.347931068471938
            sm[d, Edge(2, 4)] = 0.347931068471938
            sm[d, Edge(3, 4)] = 0.347931068471938

            mt = ModelType(Load(), MinimumMaximum(), FlowFormulation(), false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
            rd = RoutingData(g, k, nothing, mt)

            @testset "flow_routing_to_path" begin
                @test_throws ErrorException path_routing_to_flow(rd, sm)
                path_flows, edge_flows, paths = flow_routing_to_path(rd, sm)

                @test length(path_flows) == 1
                @test length(edge_flows) == 1
                @test length(paths) == 3

                for p in paths_d
                    @test p in paths
                end

                @test length(path_flows[d]) == 3
                @test length(edge_flows[d]) == 5

                @test path_flows[d][1] ≈ 0.30413786305612406
                @test path_flows[d][2] ≈ 0.347931068471938
                @test path_flows[d][3] ≈ 0.347931068471938
            end

            @testset "routing_to_matrix" begin
                r = Routing(rd, sm)

                mat = routing_to_matrix(r)
                @test mat[1, 1] ≈ 0.30413786305612406
                @test mat[1, 2] ≈ 0.347931068471938
                @test mat[1, 3] ≈ 0.347931068471938
            end
        end

        @testset "Path solution" begin
            # Very similar solution to the flow one.
            sm = JuMP.Containers.SparseAxisArray{Float64, 2, Tuple{Edge{Int}, Int}}(Dict())
            sm[d, 1] = 0.30413786305612406
            sm[d, 2] = 0.347931068471938
            sm[d, 3] = 0.347931068471938

            mt = ModelType(Load(), MinimumMaximum(), PathFormulation(), false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
            rd = RoutingData(g, k, nothing, mt)

            @testset "path_routing_to_flow" begin
                @test_throws ErrorException flow_routing_to_path(rd, sm)
                path_flows, edge_flows = path_routing_to_flow(rd, sm)

                @test length(path_flows) == 1
                @test length(edge_flows) == 1

                @test length(path_flows[d]) == 3
                @test length(edge_flows[d]) == 5

                @test path_flows[d][1] ≈ 0.30413786305612406
                @test path_flows[d][2] ≈ 0.347931068471938
                @test path_flows[d][3] ≈ 0.347931068471938
            end

            @testset "routing_to_matrix" begin
                r = Routing(rd, sm)

                mat = routing_to_matrix(r)
                @test mat[1, 1] ≈ 0.30413786305612406
                @test mat[1, 2] ≈ 0.347931068471938
                @test mat[1, 3] ≈ 0.347931068471938
            end
        end
    end

    @testset "Graph 3" begin # Solution from the "basic" graph in the examples, the solution has a loop.
        g = MetaDiGraph()

        add_vertex!(g, :name, "A")
        add_vertex!(g, :name, "B")
        add_vertex!(g, :name, "C")
        add_vertex!(g, :name, "D")
        add_vertex!(g, :name, "E")
        add_vertex!(g, :name, "F")

        add_edge!(g, 1, 2, :capacity, 1.0)
        add_edge!(g, 2, 3, :capacity, 0.9)
        add_edge!(g, 2, 6, :capacity, 1.0)
        add_edge!(g, 3, 4, :capacity, 1.0)
        add_edge!(g, 3, 6, :capacity, 1.0)
        add_edge!(g, 5, 6, :capacity, 1.0)

        add_edge!(g, 2, 1, :capacity, 1.0) # Duplicate edges.
        add_edge!(g, 3, 2, :capacity, 0.9)
        add_edge!(g, 6, 2, :capacity, 1.0)
        add_edge!(g, 4, 3, :capacity, 1.0)
        add_edge!(g, 6, 3, :capacity, 1.0)
        add_edge!(g, 6, 5, :capacity, 1.0)

        paths_all = [
            [Edge(1, 2), Edge(2, 3)],
            [Edge(1, 2), Edge(2, 3), Edge(3, 6)],
            [Edge(1, 2), Edge(2, 6)],
            [Edge(4, 3), Edge(3, 2), Edge(2, 1)],
            [Edge(4, 3), Edge(3, 2), Edge(2, 6), Edge(6, 5)],
            [Edge(4, 3), Edge(3, 6), Edge(6, 5)],
            [Edge(5, 6), Edge(6, 3), Edge(3, 2)],
            [Edge(5, 6), Edge(6, 2)],
            [Edge(6, 2), Edge(2, 3), Edge(3, 4)],
            [Edge(6, 3), Edge(3, 4)]
        ]

        k = SimpleDiGraph(6)
        add_edge!(k, 1, 3)
        add_edge!(k, 1, 6)
        add_edge!(k, 4, 1)
        add_edge!(k, 4, 5)
        add_edge!(k, 5, 2)
        add_edge!(k, 6, 4)
        d1 = Edge(1, 3)
        d2 = Edge(1, 6)
        d3 = Edge(4, 1)
        d4 = Edge(4, 5)
        d5 = Edge(5, 2)
        d6 = Edge(6, 4)
        dm = Dict{Edge{Int}, Float64}(d1 => 0.5, d2 => 0.5, d3 => 0.5, d4 => 0.5, d5 => 0.5, d6 => 0.5)

        paths_d1 = [[Edge(1, 2), Edge(2, 3)]]
        paths_d2 = [[Edge(1, 2), Edge(2, 3), Edge(3, 6)], [Edge(1, 2), Edge(2, 6)]]
        paths_d3 = [[Edge(4, 3), Edge(3, 2), Edge(2, 1)]]
        paths_d4 = [[Edge(4, 3), Edge(3, 2), Edge(2, 6), Edge(6, 5)], [Edge(4, 3), Edge(3, 6), Edge(6, 5)]]
        paths_d5 = [[Edge(5, 6), Edge(6, 3), Edge(3, 2)], [Edge(5, 6), Edge(6, 2)]]
        paths_d6 = [[Edge(6, 2), Edge(2, 3), Edge(3, 4)], [Edge(6, 3), Edge(3, 4)]]

        @testset "Flow solution" begin
            # Taken from an actual solution, numerical difficulties not removed.
            sm = JuMP.Containers.DenseAxisArray{Float64}(undef, edges(k), edges(g))

            sm[d1, Edge(1, 2)] = 1.0
            sm[d1, Edge(2, 1)] = 0.0
            sm[d1, Edge(2, 3)] = 0.9999988912829403
            sm[d1, Edge(2, 6)] = 2.120611460099956e-6
            sm[d1, Edge(3, 2)] = 0.0
            sm[d1, Edge(3, 4)] = 0.0
            sm[d1, Edge(3, 6)] = 0.0
            sm[d1, Edge(4, 3)] = 0.0
            sm[d1, Edge(5, 6)] = 1.1094507307655902e-8
            sm[d1, Edge(6, 2)] = 1.0122885971469296e-6
            sm[d1, Edge(6, 3)] = 1.1087170597439012e-6
            sm[d1, Edge(6, 5)] = 1.1094507307655902e-8

            sm[d2, Edge(1, 2)] = 1.0
            sm[d2, Edge(2, 1)] = 0.0
            sm[d2, Edge(2, 3)] = 0.3177048579857707
            sm[d2, Edge(2, 6)] = 0.7726791282105232
            sm[d2, Edge(3, 2)] = 0.09038398659069616
            sm[d2, Edge(3, 4)] = 2.476831093836052e-7
            sm[d2, Edge(3, 6)] = 0.22732087178947685
            sm[d2, Edge(4, 3)] = 2.476831093836052e-7
            sm[d2, Edge(5, 6)] = 0.0
            sm[d2, Edge(6, 2)] = 0.0
            sm[d2, Edge(6, 3)] = 0.0
            sm[d2, Edge(6, 5)] = 0.0

            sm[d3, Edge(1, 2)] = 0.0
            sm[d3, Edge(2, 1)] = 1.0
            sm[d3, Edge(2, 3)] = 1.2410080677635113e-6
            sm[d3, Edge(2, 6)] = 8.844169471686035e-7
            sm[d3, Edge(3, 2)] = 0.9999987504657342
            sm[d3, Edge(3, 4)] = 0.0
            sm[d3, Edge(3, 6)] = 3.3707368767873062e-6
            sm[d3, Edge(4, 3)] = 1.0
            sm[d3, Edge(5, 6)] = 0.0
            sm[d3, Edge(6, 2)] = 3.374629109674882e-6
            sm[d3, Edge(6, 3)] = 8.805247142810213e-7
            sm[d3, Edge(6, 5)] = 0.0

            sm[d4, Edge(1, 2)] = 3.087618250488776e-7
            sm[d4, Edge(2, 1)] = 3.087618250488776e-7
            sm[d4, Edge(2, 3)] = 0.08955509114375726
            sm[d4, Edge(2, 6)] = 0.2343737049053851
            sm[d4, Edge(3, 2)] = 0.3239280973532021
            sm[d4, Edge(3, 4)] = 0.0
            sm[d4, Edge(3, 6)] = 0.76562769985842
            sm[d4, Edge(4, 3)] = 1.0
            sm[d4, Edge(5, 6)] = 0.0
            sm[d4, Edge(6, 2)] = 6.986957449624771e-7
            sm[d4, Edge(6, 3)] = 7.063964009965341e-7
            sm[d4, Edge(6, 5)] = 1.0

            sm[d5, Edge(1, 2)] = 0.0
            sm[d5, Edge(2, 1)] = 0.0
            sm[d5, Edge(2, 3)] = 0.0
            sm[d5, Edge(2, 6)] = 0.0
            sm[d5, Edge(3, 2)] = 0.2126523775599113
            sm[d5, Edge(3, 4)] = 0.0
            sm[d5, Edge(3, 6)] = 6.99368822994739e-7
            sm[d5, Edge(4, 3)] = 0.0
            sm[d5, Edge(5, 6)] = 1.0
            sm[d5, Edge(6, 2)] = 0.7873476224400887
            sm[d5, Edge(6, 3)] = 0.2126530769949393
            sm[d5, Edge(6, 5)] = 0.0

            sm[d6, Edge(1, 2)] = 0.0
            sm[d6, Edge(2, 1)] = 0.0
            sm[d6, Edge(2, 3)] = 0.3037925494376403
            sm[d6, Edge(2, 6)] = 0.0
            sm[d6, Edge(3, 2)] = 0.09114078583113316
            sm[d6, Edge(3, 4)] = 1.0
            sm[d6, Edge(3, 6)] = 0.0
            sm[d6, Edge(4, 3)] = 0.0
            sm[d6, Edge(5, 6)] = 0.0
            sm[d6, Edge(6, 2)] = 0.21265176354084359
            sm[d6, Edge(6, 3)] = 0.7873482364591564
            sm[d6, Edge(6, 5)] = 0.0

            mt = ModelType(Load(), MinimumMaximum(), FlowFormulation(), false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
            rd = RoutingData(g, k, nothing, mt)

            @testset "flow_routing_to_path" begin
                @test_throws ErrorException path_routing_to_flow(rd, sm)
                path_flows, edge_flows, paths = flow_routing_to_path(rd, sm)

                @test length(path_flows) == 6
                @test length(edge_flows) == 6
                @test length(paths) == length(paths_all)

                for p in paths_all
                    @test p in paths
                end

                @test length(path_flows[d1]) == 1
                @test length(edge_flows[d1]) == 2
                @test length(path_flows[d2]) == 2
                @test length(edge_flows[d2]) == 4
                @test length(path_flows[d3]) == 1
                @test length(edge_flows[d3]) == 3
                @test length(path_flows[d4]) == 2
                @test length(edge_flows[d4]) == 5
                @test length(path_flows[d5]) == 2
                @test length(edge_flows[d5]) == 4
                @test length(path_flows[d6]) == 2
                @test length(edge_flows[d6]) == 4

                for p in keys(path_flows[d1])
                    @test paths[p] in paths_d1
                end
                for p in keys(path_flows[d2])
                    @test paths[p] in paths_d2
                end
                for p in keys(path_flows[d3])
                    @test paths[p] in paths_d3
                end
                for p in keys(path_flows[d4])
                    @test paths[p] in paths_d4
                end
                for p in keys(path_flows[d5])
                    @test paths[p] in paths_d5
                end
                for p in keys(path_flows[d6])
                    @test paths[p] in paths_d6
                end
            end

            @testset "routing_to_matrix" begin
                r = Routing(rd, sm)

                mat = routing_to_matrix(r)
                for i in 1:6
                    @test sum(mat[i, :]) ≈ 1.0 atol=1.0e-5
                end
            end
        end
    end
end
