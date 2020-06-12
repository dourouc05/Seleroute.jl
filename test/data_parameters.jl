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

@testset "Data: helpers" begin
    @testset "remove_unreachable_nodes!" begin
        @test ne(g_unreachable) == 2
        @test nv(g_unreachable) == 4
        @test remove_unreachable_nodes!(copy(g_unreachable)) == 1
        @test ne(g_unreachable) == 2
        @test nv(g_unreachable) == 4
    end

    @testset "remove_unsatisfiable_demands!" begin
        @test ne(g_unreachable) == 2
        @test nv(g_unreachable) == 4
        @test ne(k_unreachable) == 1
        @test remove_unsatisfiable_demands!(copy(g_unreachable), copy(k_unreachable)) == 1
        @test ne(g_unreachable) == 2
        @test nv(g_unreachable) == 4
        @test ne(k_unreachable) == 1
    end
end

@testset "Data: parameters" begin
    @testset "Undirected graph" begin
        mt = ModelType(Load, MinimumTotal, PathFormulation, false, CuttingPlane, ObliviousUncertainty, UncertainDemand)
        @test_throws ErrorException RoutingData(copy(ug), copy(k), ECOS.Optimizer, mt)
        @test_throws ErrorException RoutingData(copy(g), copy(uk), ECOS.Optimizer, mt)
        @test_throws ErrorException RoutingData(copy(ug), copy(uk), ECOS.Optimizer, mt)
    end

    @testset "Inexisting folder" begin
        mt = ModelType(Load, MinimumTotal, PathFormulation, false, CuttingPlane, ObliviousUncertainty, UncertainDemand)
        @test_throws ErrorException RoutingData(copy(g), copy(k), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
    end

    @testset "Unsatisfiable or unreachable parts" begin
        mt = ModelType(Load, MinimumTotal, PathFormulation, false, CuttingPlane, ObliviousUncertainty, UncertainDemand)
        @test_throws ErrorException RoutingData(copy(g_unreachable), copy(k), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
        @test_throws ErrorException RoutingData(copy(g), copy(k_unreachable), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
    end

    @testset "Parameter compatibility" begin
        for edge_obj in subtypes(EdgeWiseObjectiveFunction)
            for agg_obj in subtypes(AggregationObjectiveFunction)
                for type in subtypes(FormulationType)
                    for cg in [true, false]
                        for algo in subtypes(AlgorithmChoice)
                            for unc in subtypes(UncertaintyHandling)
                                for uncparams in subtypes(UncertainParameters)
                                    @testset "ModelType($edge_obj, $agg_obj, $type, $cg, $algo, $unc, $uncparams)" begin
                                        # Special cases.
                                        if edge_obj == AlphaFairness
                                            # TODO: provide a value for α and run the same tests as below.
                                            
                                        # Failures.
                                        elseif uncparams == UncertainCapacity && edge_obj in [Load, KleinrockLoad, FortzThorupLoad]
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                        elseif type == FlowFormulation && cg
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                        elseif (unc != ObliviousUncertainty && unc != RobustUncertainty) && algo != Automatic
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                        elseif unc == NoUncertaintyHandling && uncparams != NoUncertainty
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                        elseif unc == NoUncertaintyHandling && uncparams != NoUncertainty
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                        elseif uncparams == UncertainCapacity
                                            @test_throws ErrorException ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)

                                        # Successes.
                                        else
                                            # The ModelType object must build properly.
                                            mt = ModelType(edge_obj, agg_obj, type, cg, algo, unc, uncparams)

                                            @test mt.edge_obj == edge_obj()
                                            @test mt.agg_obj == agg_obj()
                                            @test mt.type == type()
                                            @test mt.cg == cg
                                            @test mt.unc == unc()
                                            @test mt.uncparams == uncparams()

                                            # Automatic parameter detection.
                                            if (unc == ObliviousUncertainty || unc == RobustUncertainty) && algo == Automatic
                                                @test mt.algo != algo()
                                            else
                                                @test mt.algo == algo()
                                            end

                                            # Test that RoutingData builds properly.
                                            rd = RoutingData(copy(g), copy(k), ECOS.Optimizer, mt)

                                            @test rd.g == g
                                            @test rd.k == k
                                            @test rd.solver == ECOS.Optimizer
                                            @test rd.name == ""
                                            @test rd.time_precompute_ms > 0.0
                                            @test rd.model_type == mt
                                            @test rd.sub_model_type == mt
                                            @test length(rd.traffic_matrix) == 0

                                            @test length(edges(rd)) == 4
                                            @test length(vertices(rd)) == 4
                                            @test length(demands(rd)) == 1
                                            @test n_nodes(rd) == 4
                                            @test n_edges(rd) == 4
                                            @test n_demands(rd) == 1
                                            @test n_paths(rd) == (type == PathFormulation ? 2 : 0)

                                            if type == PathFormulation
                                                @test [Edge(1, 2), Edge(2, 4)] in rd.paths_edges
                                                @test [Edge(1, 3), Edge(3, 4)] in rd.paths_edges
                                                @test length(rd.demand_to_path_ids[Edge(1, 4)]) == 2
                                                @test rd.path_id_to_demand[1] == Edge(1, 4)
                                                @test rd.path_id_to_demand[2] == Edge(1, 4)
                                            else
                                                @test type == FlowFormulation
                                                @test length(rd.paths_edges) == 0
                                                @test length(rd.demand_to_path_ids) == 0
                                                @test length(rd.path_id_to_demand) == 0
                                            end

                                            # Warning when generating too many paths.
                                            if type == FlowFormulation
                                                @test_nowarn RoutingData(g, k, ECOS.Optimizer, mt, npaths=1)
                                            elseif cg == false
                                                @test type == PathFormulation
                                                wrnmsg = "The limit of paths to generate (1) has been reached; the result will not be " *
                                                      "guaranteed to be optimum. Switch to a column-generation formulation for exact results."
                                                @test_logs (:warn, wrnmsg) RoutingData(g, k, ECOS.Optimizer, mt, npaths=1)
                                            end

                                            # TODO: what about the submodel type?
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
