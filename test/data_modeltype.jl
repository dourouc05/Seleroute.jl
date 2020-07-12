@testset "ModelType: parameter compatibility" begin
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

                                                @test mt !== nothing
                                                @test typeof(mt.edge_obj) == edge_obj
                                                @test mt.edge_obj.α == α
                                                @test mt.edge_obj.force_power_cone == pc
                                                @test mt.agg_obj == agg_obj()
                                                @test mt.type == type()
                                                @test mt.cg == cg
                                                @test mt.unc == unc()
                                                @test mt.uncparams == uncparams()

                                                # Automatic detection of the parameter.
                                                if (unc == ObliviousUncertainty || unc == RobustUncertainty) && algo == Automatic
                                                    @test mt.algo != algo()
                                                else
                                                    @test mt.algo == algo()
                                                end
                                            end
                                        end
                                    else
                                        # Generic case.
                                        mt = ModelType(edge_obj(), agg_obj(), type(), cg, algo(), unc(), uncparams())

                                        # The model type object should match the arguments.
                                        @test mt.edge_obj == edge_obj()
                                        @test mt.agg_obj == agg_obj()
                                        @test mt.type == type()
                                        @test mt.cg == cg
                                        @test mt.unc == unc()
                                        @test mt.uncparams == uncparams()

                                        # Automatic detection of the parameter.
                                        if (unc == ObliviousUncertainty || unc == RobustUncertainty) && algo == Automatic
                                            @test mt.algo != algo()
                                        else
                                            @test mt.algo == algo()
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
