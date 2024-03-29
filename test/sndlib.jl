@testset "SNDlib native format" begin
    @testset "Basic instance" begin
        instance = """
            ?SNDlib native format; type: network; version: 1.0
            # network test
            
            NODES (
                N1 ( 1.00 2.00 )
                N2 (-1.00 -2.0 )
                N3 ( 1.00 -2.00)
                N4 (-1.00 -2.00)
            )
            
            LINKS (
                L1_N1_N2 ( N1 N2 ) 0.00 0.00 0.00 0.00 ( 42.0 84 )
                L2_N1_N3 ( N1 N3 ) 0.00 0.00 0.00 0.00 ( -420 56.0 84.0 -112 )
                L3_N1_N4 ( N1 N4 ) 0.00 0.00 0.00 0.00 ( UNLIMITED 0.0 )
            )
            
            DEMANDS (
                D1_N2_N3 ( N2 N3 ) 1 42.00 UNLIMITED
                D2_N3_N4 ( N3 N4 ) 1 42.00 5
            )
            
            ADMISSIBLE_PATHS ( 
            )
        """
        g_edges = [Edge(1, 2), Edge(1, 3), Edge(1, 4)]
        k_edges = [Edge(2, 3), Edge(3, 4)]

        g, k = loadgraph(
            IOBuffer(instance), "useless graph name", SNDlibNativeFormat())

        @testset "Graph size" begin
            @testset "Topology graph g has the right number of vertices and edges" begin
                @test nv(g) == 4
                @test ne(g) == length(g_edges)
            end
            @testset "Demand graph k has the right number of vertices and edges" begin
                @test nv(k) == 4
                @test ne(k) == length(k_edges)
            end
        end

        @testset "Vertex existence" begin
            @testset "Topology graph g has vertex #$(i)" for i in 1:4
                @test has_vertex(g, i)
            end
            @testset "Demand graph k has vertex #$(i)" for i in 1:4
                @test has_vertex(k, i)
            end
        end

        @testset "Edge existence" begin
            @testset "Topology graph g has $(e)" for e in g_edges
                @test has_edge(g, e)
            end
            @testset "Demand graph k has $(e)" for e in k_edges
                @test has_edge(k, e)
            end
        end

        @testset "Graph properties" begin
            @testset "Topology graph g has the right name" begin
                @test get_prop(g, :graph_name) == "test"
            end
        end

        @testset "Node properties" begin
            @testset "The node $(i) of the topology graph g has the right properties" for i in 1:4
                @test get_prop(g, i, :vertex_id) == "N$(i)"
    
                @test get_prop(g, i, :latitude) == ifelse(i % 2 == 1, 1.0, -1.0)
                @test get_prop(g, i, :longitude) == ifelse(i == 1, 2.0, -2.0)
            end
            @testset "The node $(i) of the demand graph k has the right properties" for i in 1:4
                @test get_prop(k, i, :vertex_id) == "N$(i)"
            end
        end

        @testset "Edge properties" begin
            @testset "The $(e) (#$(i)) of the topology graph g has the right properties" for (i, e) in enumerate(g_edges)
                @test get_prop(g, e, :edge_id) == "L$(i)_N$(src(e))_N$(dst(e))"
                @test get_prop(g, e, :preinstalled_capacity) == 0.0
                @test get_prop(g, e, :preinstalled_cost) == 0.0
                @test get_prop(g, e, :routing_cost) == 0.0
                @test get_prop(g, e, :setup_cost) == 0.0

                capacity_cost = 
                    if i == 1
                        Dict{Union{Missing, Float64, Int}, Union{Float64, Int}}(42.0 => 84)
                    elseif i == 2 
                        Dict{Union{Missing, Float64, Int}, Union{Float64, Int}}(-420 => 56.0, 84.0 => -112)
                    elseif i == 3
                        Dict{Union{Missing, Float64, Int}, Union{Float64, Int}}(missing => 0.0)
                    end
                @test get_prop(g, e, :capacity_cost) == capacity_cost
            end
            @testset "The $(e) (#$(i)) of the demand graph k has the right properties" for (i, e) in enumerate(k_edges)
                @test get_prop(k, e, :demand_id) == "D$(i)_N$(src(e))_N$(dst(e))"
                @test get_prop(k, e, :routing_unit) == 1
                @test get_prop(k, e, :demand_value) == 42.0
                @test get_prop(k, e, :max_path_length) === ifelse(i == 1, missing, 5)
            end
        end
    end
    
    @testset "Meta instance" begin
        instance = """
            ?SNDlib native format; type: network; version: 1.0
            # network test

            META (
            granularity = 6month
            time = 2000
            unit = MBITPERSEC
            origin = total demand 3Tbps
            )
            
            NODES (
                N1 ( 1.00 2.00 )
                N2 (-1.00 -2.0 )
                N3 ( 1.00 -2.00)
                N4 (-1.00 -2.00)
            )
            
            LINKS (
                L1_N1_N2 ( N1 N2 ) 0.00 0.00 0.00 0.00 ( 42.0 84 )
                L2_N1_N3 ( N1 N3 ) 0.00 0.00 0.00 0.00 ( -420 56.0 84.0 -112 )
                L3_N1_N4 ( N1 N4 ) 0.00 0.00 0.00 0.00 ( UNLIMITED 0.0 )
            )
            
            DEMANDS (
                D1_N2_N3 ( N2 N3 ) 1 42.00 UNLIMITED
                D2_N3_N4 ( N3 N4 ) 1 42.00 5
            )
            
            ADMISSIBLE_PATHS ( 
            )
        """

        g, k = loadgraph(
            IOBuffer(instance), "useless graph name", SNDlibNativeFormat())
        
        @testset "Graph properties" begin
            @testset "Topology graph g has the right name" begin
                @test get_prop(g, :graph_name) == "test"
            end
            @testset "Demand graph k has the right name" begin
                @test get_prop(g, :graph_name) == "test"
            end
            @testset "Topology graph g has the right meta fields" begin
                @test get_prop(g, :granularity) == "6month"
                @test get_prop(g, :time) == "2000"
                @test get_prop(g, :unit) == "MBITPERSEC"
                @test get_prop(g, :origin) == "total demand 3Tbps"
            end
        end
    end

    @testset "Link with empty capacity-cost list" begin
        # The problem was the number of spaces between the parentheses.
        instance = """
            ?SNDlib native format; type: network; version: 1.0
            # network test
            
            NODES (
                N1 (1.0 2.0)
                N2 (1.0 2.0)
            )
            
            LINKS (
                L1_N1_N2 ( N1 N2 ) 600.00 1200.00 0.00 0.00 (  )
                L ( N2 N1 ) 600.00 1200.00 0.00 0.00 ( )
            )
        """

        g, k = loadgraph(
            IOBuffer(instance), "useless graph name", SNDlibNativeFormat())

        @testset "Graph size" begin
            @testset "Topology graph g has the right number of vertices and edges" begin
                @test nv(g) == 2
                @test ne(g) == 2
            end
            @testset "Demand graph k has the right number of vertices and edges" begin
                @test nv(k) == 2
                @test ne(k) == 0
            end
        end
    end

    @testset "Vertex with a dot in its name" begin
        instance = """
            ?SNDlib native format; type: network; version: 1.0
            
            NODES (
                N.1 (1.0 2.0)
                N.2 (1.0 2.0)
            )
            
            LINKS (
                L.1_N.1_N.2 ( N.1 N.2 ) 600.00 1200.00 0.00 0.00 (  )
                L. ( N.2 N.1 ) 600.00 1200.00 0.00 0.00 ( )
            )
        """

        g, k = loadgraph(
            IOBuffer(instance), "useless graph name", SNDlibNativeFormat())

        @testset "Graph size" begin
            @testset "Topology graph g has the right number of vertices and edges" begin
                @test nv(g) == 2
                @test ne(g) == 2
            end
            @testset "Demand graph k has the right number of vertices and edges" begin
                @test nv(k) == 2
                @test ne(k) == 0
            end
        end
    end
    
    @testset "Admissible paths" begin
        instance = """
            ?SNDlib native format; type: network; version: 1.0
            
            NODES (
                N1 ( 1.00 2.00 )
                N2 ( 2.00 1.00 )
            )
            
            LINKS (
                L1 ( N1 N2 ) 0.0 0.0 0.0 0.0 ( )
            )
            
            DEMANDS (
                D1 ( N1 N2 ) 1 42.00 UNLIMITED
                D2 ( N2 N1 ) 1 42.00 UNLIMITED
            )
            
            ADMISSIBLE_PATHS ( 
                D1 (
                    P_0 ( L1 )
                )
                D2 (
                    P_0 ( L1 )
                )
            )
        """

        @testset "Errors when asked to" begin
            @test_throws ErrorException loadgraph(
                IOBuffer(instance), "useless graph name",
                SNDlibNativeFormat(), error_on_admissible_path=true)
        end

        @testset "Does not error when not asked to" begin
            g, k = loadgraph(
                IOBuffer(instance), "useless graph name",
                SNDlibNativeFormat(), error_on_admissible_path=false)
            @test nv(g) == 2
        end
    end
end
