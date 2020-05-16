@testset "Topology: topology_to_graphs" begin
    @testset "Directed" begin
        nodes_matrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
        edges_matrix = [[1, 2, 1] [2, 3, 2] [2, 6, 2] [3, 4, 3] [3, 6, 3] [5, 6, 5]]'
        traffic_matrix = [[1, 3, .5] [1, 6, .5] [4, 1, .5] [4, 5, .5] [5, 2, .5] [6, 4, .5]]'
        t = Topology(nodes_matrix, edges_matrix, traffic_matrix)

        g, k = topology_to_graphs(t)

        @test nv(g) == 6
        @test ne(g) == 12 # Directed graph
        @test nv(k) == 6
        @test ne(k) == 6

        for i in 1:6
            @test get_prop(g, i, :name) == string('A' + i - 1)
            @test get_prop(k, i, :name) == string('A' + i - 1)
        end
        for (s, t) in [(1, 2), (2, 3), (2, 6), (3, 4), (3, 6), (5, 6)]
            @test has_edge(g, s, t)
            @test get_prop(g, s, t, :capacity) == s

            @test has_edge(g, t, s)
            @test get_prop(g, t, s, :capacity) == s
        end
        for (s, t) in [(1, 3), (1, 6), (4, 1), (4, 5), (5, 2), (6, 4)]
            @test has_edge(k, s, t)
            @test ! has_edge(k, t, s)

            @test get_prop(k, s, t, :demand) == 0.5
        end
    end

    @testset "Undirected" begin
        nodes_matrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
        edges_matrix = [[1, 2, 1] [2, 3, 2] [2, 6, 2] [3, 4, 3] [3, 6, 3] [5, 6, 5]]'
        traffic_matrix = [[1, 3, .5] [1, 6, .5] [4, 1, .5] [4, 5, .5] [5, 2, .5] [6, 4, .5]]'
        t = Topology(nodes_matrix, edges_matrix, traffic_matrix)

        g, k = topology_to_graphs(t, make_network_undirected=false)

        @test nv(g) == 6
        @test ne(g) == 6 # Undirected graph
        @test nv(k) == 6
        @test ne(k) == 6

        for i in 1:6
            @test get_prop(g, i, :name) == string('A' + i - 1)
            @test get_prop(k, i, :name) == string('A' + i - 1)
        end
        for (s, t) in [(1, 2), (2, 3), (2, 6), (3, 4), (3, 6), (5, 6)]
            @test has_edge(g, s, t)
            @test get_prop(g, s, t, :capacity) == s

            @test ! has_edge(g, t, s)
        end
        for (s, t) in [(1, 3), (1, 6), (4, 1), (4, 5), (5, 2), (6, 4)]
            @test has_edge(k, s, t)
            @test ! has_edge(k, t, s)

            @test get_prop(k, s, t, :demand) == 0.5
        end
    end
end
