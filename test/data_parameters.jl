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
        mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        @test_throws ErrorException RoutingData(copy(ug), copy(k), ECOS.Optimizer, mt)
        @test_throws ErrorException RoutingData(copy(g), copy(uk), ECOS.Optimizer, mt)
        @test_throws ErrorException RoutingData(copy(ug), copy(uk), ECOS.Optimizer, mt)
    end

    @testset "Inexisting folder" begin
        mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        @test_throws ErrorException RoutingData(copy(g), copy(k), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
    end

    @testset "Unsatisfiable or unreachable parts" begin
        mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        @test_throws ErrorException RoutingData(copy(g_unreachable), copy(k), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
        @test_throws ErrorException RoutingData(copy(g), copy(k_unreachable), ECOS.Optimizer, mt, output_folder="/42/84/bullshit/")
    end

    @testset "Parameter compatibility" begin
        function __test_mt_modeltype(mt, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
            # The model type object should match the arguments.
            @test mt.edge_obj == edge_obj()
            @test mt.agg_obj == agg_obj()
            @test mt.type == type()
            @test mt.cg == cg
            @test mt.unc == unc()
            @test mt.uncparams == uncparams()
        end

        function __test_mt_main(mt, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
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
        end

        for edge_obj in subtypes(EdgeWiseObjectiveFunction)
            for agg_obj in vcat([subtypes(s) for s in subtypes(AggregationObjectiveFunction)]...) # Assumption: only abstract types directly derive from AggregationObjectiveFunction.
                for type in subtypes(FormulationType)
                    for cg in [true, false]
                        for algo in subtypes(AlgorithmChoice)
                            for unc in subtypes(UncertaintyHandling)
                                for uncparams in subtypes(UncertainParameters)
                                    @testset "ModelType($edge_obj, $agg_obj, $type, $cg, $algo, $unc, $uncparams)" begin
                                        # Failures.
                                        if uncparams == UncertainCapacity && edge_obj in [Load, KleinrockLoad, FortzThorupLoad]
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                        elseif type == FlowFormulation && cg
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                        elseif (unc != ObliviousUncertainty && unc != RobustUncertainty) && algo != Automatic
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                        elseif unc == NoUncertaintyHandling && uncparams != NoUncertainty
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                        elseif unc == NoUncertaintyHandling && uncparams != NoUncertainty
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                        elseif uncparams == UncertainCapacity
                                            @test_throws ErrorException ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())

                                        # Successes.
                                        elseif edge_obj == AlphaFairness
                                            for α in [0.0, 0.25, 1.0]
                                                for pc in [true, false]
                                                    mt = nothing

                                                    if pc && α == 0.0
                                                        # Warn when forcing the use of a power cone when there is not available.
                                                        msg = "The power-cone formulation is forced to be used, " *
                                                              "but it is not available for α = $(α). " *
                                                              "The parameter `force_power_cone` is therefore ignored."
                                                        @test_logs (:warn, msg) mt = ModelType(AlphaFairness(α, pc), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                                    elseif pc && ! (α in [0.5, 1.5, 2.0])
                                                        # Warn when forcing the use of a power cone when there is nothing else available.
                                                        msg = "The power-cone formulation is forced to be used, " *
                                                              "but it is the only available one for α = $(α). " *
                                                              "The parameter `force_power_cone` is therefore ignored."
                                                        @test_logs (:warn, msg) mt = ModelType(AlphaFairness(α, pc), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                                    else
                                                        # Don't warn in other cases.
                                                        @test_nowarn mt = ModelType(AlphaFairness(α, pc), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                                    end

                                                    # __test_mt_modeltype
                                                    @test typeof(mt.edge_obj) == edge_obj
                                                    @test mt.edge_obj.α == α
                                                    @test mt.edge_obj.force_power_cone == pc
                                                    @test mt.agg_obj == agg_obj()
                                                    @test mt.type == type()
                                                    @test mt.cg == cg
                                                    @test mt.unc == unc()
                                                    @test mt.uncparams == uncparams()

                                                    __test_mt_main(mt, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                                end
                                            end
                                        else
                                            # Generic case.
                                            mt = ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())
                                            __test_mt_modeltype(mt, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
                                            __test_mt_main(mt, edge_obj, agg_obj, type, cg, algo, unc, uncparams)

                                            # TODO: what about the submodel type? Testing that it is essentially the same as the main object, also try to generate a different one.
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
