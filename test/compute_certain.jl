function __testset_nouncertainty_shared(r::RoutingSolution, rd::RoutingData, dm)
    @test r !== nothing
    @test r.data == rd
    @test r.n_iter == 0
    @test r.n_matrices == 1
    @test r.n_cuts == 0
    @test r.n_columns == 0
    @test r.time_precompute_ms > 0.0
    @test r.time_solve_ms > 0.0
    @test r.time_export_ms == 0.0
    @test length(r.objectives) == 1
    @test length(r.matrices) == 1
    @test length(r.routings) == 1
    @test r.master_model !== nothing
    @test r.matrices[1] == dm
end

function __testset_nouncertainty_minmax(edge_obj, type, opt, g, paths, k, d, dm)
    @testset "Objective aggregation: minimum maximum" begin
        mt = ModelType(edge_obj, MinimumMaximum(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)

        r = nothing
        if typeof(edge_obj) == AlphaFairness && edge_obj.α != 0.0
            msg = "The objective function $(edge_obj) does not support minimisation. Proceed with caution."
            @test_logs (:warn, msg) r = compute_routing(rd)
        else
            @test_nowarn r = compute_routing(rd)
        end

        __testset_nouncertainty_shared(r, rd, dm)
        if edge_obj == Load()
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
        elseif edge_obj == KleinrockLoad()
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ 1.333333 atol=1.0e-5
        elseif edge_obj == FortzThorupLoad()
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ 1.047619 atol=1.0e-5
        elseif edge_obj == Load()
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α in [0.0, 0.5, 1.0, 1.5, 2.0]
            # Minimising the total fairness, which is usually nonsensical
            # (hence no objective value check).
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
        else
            println(r.objectives)
            @test_broken true == false
        end

        if termination_status(r.master_model.model) == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].paths) == length(paths) # Number of paths
            for p in paths
                @test p in r.routings[1].paths
            end
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges
            @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=2.0e-3 # Relaxed tolerance needed for some cases of α-fairness.
        end
    end

    @testset "Objective aggregation: maximum minimum" begin
        mt = ModelType(edge_obj, MaximumMinimum(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)
        r = compute_routing(rd)

        __testset_nouncertainty_shared(r, rd, dm)
        if edge_obj == Load()
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
        elseif edge_obj == KleinrockLoad()
            @test termination_status(r.master_model.model) in [MOI.DUAL_INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED]
        elseif edge_obj == FortzThorupLoad()
            @test termination_status(r.master_model.model) in [MOI.DUAL_INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED]
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0
            # Exactly as if optimising the load! Except that the power-cone model
            # may pose numerical problems.
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.5
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 1.511857 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.0
            @test termination_status(r.master_model.model) == MOI.OPTIMAL
            @test r.objectives[1] ≈ -0.559636 atol=1.0e-4 # Looks like there are slight variations betweens flows and paths.
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.5
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                # Neither SCS nor ECOS can find the optimum (but CPLEX does).
                @test r.objectives[1] ≈ -1.322875 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 2.0
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ -1.749999 atol=1.0e-5
            end
        else
            println(r.objectives)
            @test_broken true == false
        end

        if termination_status(r.master_model.model) == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].paths) == length(paths) # Number of paths
            for p in paths
                @test p in r.routings[1].paths
            end
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 5] # Number of edges
            @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=1.0e-3
        end
    end
end

function __testset_nouncertainty_mintot(edge_obj, type, opt, g, paths, k, d, dm)
    @testset "Objective aggregation: minimum total" begin
        mt = ModelType(edge_obj, MinimumTotal(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)

        r = nothing
        if typeof(edge_obj) == AlphaFairness && edge_obj.α != 0.0
            msg = "The objective function $(edge_obj) does not support minimisation. Proceed with caution."
            @test_logs (:warn, msg) r = compute_routing(rd)
        else
            @test_nowarn r = compute_routing(rd)
        end
        @test r !== nothing

        __testset_nouncertainty_shared(r, rd, dm)
        if edge_obj == Load() || (typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0)
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 2.0 atol=1.0e-5
            end
        elseif edge_obj == KleinrockLoad()
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 5.952135 atol=1.0e-5
            end
        elseif edge_obj == FortzThorupLoad()
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 4.666666 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.5
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] ≈ 0.000000 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α in [0.5, 1.0, 1.5, 2.0]
            # Minimising the total fairness, which is usually nonsensical.
            @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if termination_status(r.master_model.model) == MOI.OPTIMAL
                @test r.objectives[1] <= -1000.0
            end
        else
            println(r.objectives)
            @test_broken true == false
        end

        if termination_status(r.master_model.model) == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            # This solution only uses two paths out of three, i.e. three edges out of five.
            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].paths) in [2, 3] # Number of paths
            for p in [[Edge(1, 4)], [Edge(1, 2), Edge(2, 4)]]
                @test p in r.routings[1].paths
            end
            if length(r.routings[1].paths) >= 3
                @test [Edge(1, 3), Edge(3, 4)] in r.routings[1].paths
            end
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges
            @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=1.0e-3
        end
    end

    # @testset "Objective aggregation: maximum total" begin
    #     mt = ModelType(edge_obj, MaximumTotal(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
    #     rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)
    #     r = compute_routing(rd)
    #
    #     # __testset_nouncertainty_shared(r, rd, dm)
    #     # if edge_obj == Load() || (typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0)
    #     #     @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
    #     #     if termination_status(r.master_model.model) == MOI.OPTIMAL
    #     #         @test r.objectives[1] ≈ 2.0 atol=1.0e-5
    #     #     end
    #     # elseif edge_obj == KleinrockLoad()
    #     #     @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
    #     #     if termination_status(r.master_model.model) == MOI.OPTIMAL
    #     #         @test r.objectives[1] ≈ 5.952135 atol=1.0e-5
    #     #     end
    #     # elseif edge_obj == FortzThorupLoad()
    #     #     @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
    #     #     if termination_status(r.master_model.model) == MOI.OPTIMAL
    #     #         @test r.objectives[1] ≈ 4.666666 atol=1.0e-5
    #     #     end
    #     # elseif typeof(edge_obj) == AlphaFairness && edge_obj.α in [0.5, 1.0, 1.5, 2.0]
    #     #     # Minimising the total fairness, which is usually nonsensical.
    #     #     @test termination_status(r.master_model.model) in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
    #     #     if termination_status(r.master_model.model) == MOI.OPTIMAL
    #     #         @test r.objectives[1] <= -1000.0
    #     #     end
    #     # else
    #     #     println(r.objectives)
    #     #     @test_broken true == false
    #     # end
    #     #
    #     # if termination_status(r.master_model.model) == MOI.OPTIMAL
    #     #     # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
    #     #     # are respected, but the sum over all paths may be "slightly" off
    #     #     # (1.05, for instance).
    #     #
    #     #     # This solution only uses two paths out of three, i.e. three edges out of five.
    #     #     @test r.routings[1].data == rd
    #     #     @test length(r.routings[1].demands) == ne(k) # Number of demands
    #     #     @test r.routings[1].demands[1] == d
    #     #     @test length(r.routings[1].paths) in [2, 3] # Number of paths
    #     #     for p in [[Edge(1, 4)], [Edge(1, 2), Edge(2, 4)]]
    #     #         @test p in r.routings[1].paths
    #     #     end
    #     #     if length(r.routings[1].paths) >= 3
    #     #         @test [Edge(1, 3), Edge(3, 4)] in r.routings[1].paths
    #     #     end
    #     #     @test length(r.routings[1].path_flows) == ne(k) # Number of demands
    #     #     @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
    #     #     @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
    #     #     @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges
    #     #     @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=1.0e-3
    #     # end
    # end
end

function __testset_nouncertainty_mmf(edge_obj, type, opt, g, paths, k, d, dm)
    @testset "Objective aggregation: min-max fair" begin
        mt = ModelType(edge_obj, MinMaxFair(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)
        r = compute_routing(rd)

        __testset_nouncertainty_shared(r, rd, dm)
        @test isnan(r.objectives[1]) # This is a vector objective function.

        @test r.routings[1].data == rd
        @test length(r.routings[1].demands) == ne(k) # Number of demands
        @test r.routings[1].demands[1] == d
        @test length(r.routings[1].paths) <= length(paths) # Number of paths
        @test length(r.routings[1].paths) >= 1
        @test length(r.routings[1].path_flows) == ne(k) # Number of demands
        @test length(r.routings[1].path_flows[d]) in [1, 2, 3] # Number of paths
        @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
        @test length(r.routings[1].edge_flows[d]) in [1, 3, 5] # Number of edges
        @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=1.0e-5

        # Check that all paths are used, i.e. each of them sees at least 20% of the traffic.
        @test all(collect(values(r.routings[1].path_flows[d])) .>= 0.2)

        if typeof(edge_obj) != AlphaFairness # Probably due to poor accuracy.
            @test collect(values(r.routings[1].path_flows[d])) ≈ [0.2857, 0.2857, 0.4285] atol=1.0e-4
        end
    end
end
