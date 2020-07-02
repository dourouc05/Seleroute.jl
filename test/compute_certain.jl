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

        status = termination_status(r.master_model.model)
        if edge_obj == Load()
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
        elseif edge_obj == KleinrockLoad()
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ 1.333333 atol=1.0e-5
        elseif edge_obj == FortzThorupLoad()
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ 1.047619 atol=1.0e-5
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α in [0.5, 1.0, 1.5, 2.0]
            # Minimising the total fairness, which is usually nonsensical
            # (hence no objective value check).
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
        else
            println(r.objectives)
            @test_broken true == false
        end

        if status == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges

            total_flow = sum(x for x in values(r.routings[1].path_flows[d]))
            if abs(total_flow - 1.0) > 1.0e-3
                # SCS has troubles with some models (total_flow ≈ 0.998207).
                @test_broken total_flow ≈ 1.0 atol=1.0e-3
            else
                # Relaxed tolerance needed for some cases of α-fairness.
                @test total_flow ≈ 1.0 atol=1.0e-3
            end
        end
    end

    @testset "Objective aggregation: maximum minimum" begin
        mt = ModelType(edge_obj, MaximumMinimum(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)

        r = nothing
        if typeof(edge_obj) in [KleinrockLoad, FortzThorupLoad]
            msg = "The objective function $(edge_obj) does not support maximisation. Proceed with caution."
            @test_logs (:warn, msg) r = compute_routing(rd)
        else
            @test_nowarn r = compute_routing(rd)
        end

        __testset_nouncertainty_shared(r, rd, dm)

        status = termination_status(r.master_model.model)
        if edge_obj == Load()
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
        elseif edge_obj == KleinrockLoad()
            @test status in [MOI.DUAL_INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED, MOI.SLOW_PROGRESS]
        elseif edge_obj == FortzThorupLoad()
            @test status in [MOI.DUAL_INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED, MOI.SLOW_PROGRESS]
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0
            # Exactly as if optimising the load! Except that the power-cone model
            # may pose numerical problems.
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 0.571428 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.5
            # SCS has troubles with this one: it sometimes thinks it is
            # unbounded, othertimes it gives a wrong answer.
            if status in [MOI.ALMOST_DUAL_INFEASIBLE, MOI.DUAL_INFEASIBLE]
                @test_broken status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
                @test_broken r.objectives[1] ≈ 1.511857 atol=1.0e-5
            else
                @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
                if status == MOI.OPTIMAL
                    if abs(r.objectives[1] - 1.511857) < 1.0e-5
                        @test r.objectives[1] ≈ 1.511857 atol=1.0e-5
                    else
                        @test_broken r.objectives[1] ≈ 1.511857 atol=1.0e-5
                    end
                end
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.0
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ -0.559636 atol=1.0e-4 # Looks like there are slight variations betweens flows and paths.
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.5
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL && abs(r.objectives[1] + 2.645751) < 1.0e-5
                # Neither SCS nor ECOS can find the optimum.
                # CPLEX (SOCP): -1.322875
                # Mosek (SOCP and POW): -2.645751
                @test r.objectives[1] ≈ -2.645751 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 2.0
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                if abs(r.objectives[1] + 1.749999) < 1.0e-5
                    @test r.objectives[1] ≈ -1.749999 atol=1.0e-5
                else
                    # Neither SCS nor ECOS can find the optimum.
                    @test_broken r.objectives[1] ≈ -1.749999 atol=1.0e-5
                end
            end
        else
            println(r.objectives)
            @test_broken true == false
        end

        if status == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges
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

        status = termination_status(r.master_model.model)
        if edge_obj == Load() || (typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0)
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 2.0 atol=1.0e-5
            end
        elseif edge_obj == KleinrockLoad()
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 5.952135 atol=1.0e-5
            end
        elseif edge_obj == FortzThorupLoad()
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 4.666666 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.5
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 0.000000 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α in [0.5, 1.0, 1.5, 2.0]
            # Minimising the total fairness, which is usually nonsensical.
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] <= -1000.0
            end
        else
            println(r.objectives)
            @test_broken true == false
        end

        if status == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            # This solution only uses two paths out of three, i.e. three edges out of five.
            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].paths) in [2, 3] # Number of paths
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges

            total_flow = sum(x for x in values(r.routings[1].path_flows[d]))
            if abs(total_flow - 1.0) > 1.0e-3
                # SCS has troubles with this one (total_flow ≈ 0.977865 for
                # 2-fair, or even 0.998207 or 1.004190 for others).
                @test_broken total_flow ≈ 1.0 atol=1.0e-3
            else
                @test total_flow ≈ 1.0 atol=1.0e-3
            end
        end
    end

    @testset "Objective aggregation: maximum total" begin
        mt = ModelType(edge_obj, MaximumTotal(), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)
        r = nothing

        if typeof(edge_obj) != AlphaFairness && typeof(edge_obj) != Load
            msg = "The objective function $(edge_obj) does not support maximisation. Proceed with caution."
            @test_logs (:warn, msg) r = compute_routing(rd)
        else
            @test_nowarn r = compute_routing(rd)
        end

        __testset_nouncertainty_shared(r, rd, dm)

        status = termination_status(r.master_model.model)
        if edge_obj == Load()
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ 4.0 atol=1.0e-5
            end
        elseif edge_obj == KleinrockLoad() || edge_obj == FortzThorupLoad()
            # Maximising these objective functions does not really make sense.
            @test status in [MOI.INFEASIBLE_OR_UNBOUNDED, MOI.DUAL_INFEASIBLE]
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.0
            @test status == MOI.OPTIMAL
            @test r.objectives[1] ≈ 4.0 atol=1.0e-5
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 0.5
            if ! edge_obj.force_power_cone || (edge_obj.force_power_cone && status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL])
                # SCS incorrectly determines this problem is unbounded with
                # the power-cone formulation, but Mosek sees no problem
                # and yields the same solution as for the SOCP formulation.
                @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
                if status == MOI.OPTIMAL
                    @test r.objectives[1] ≈ 8.326663 atol=1.0e-5
                end
            else
                @test_broken status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
                @test_broken r.objectives[1] ≈ 8.326663 atol=1.0e-5
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.0
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                @test r.objectives[1] ≈ -2.214 atol=1.0e-3
                # Mosek: -2.214330
                # ECOS: -2.214426.
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 1.5
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                # SCS has numerical problems here: it outputs
                # -1.597694850698785e-7, with a solution that is not really
                # feasible.
                if abs(r.objectives[1] + 12.696022) <= 1.0e-5
                    @test r.objectives[1] ≈ -12.696022 atol=1.0e-5
                else
                    @test_broken r.objectives[1] ≈ -12.696022 atol=1.0e-5
                end
            end
        elseif typeof(edge_obj) == AlphaFairness && edge_obj.α == 2.0
            @test status in [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
            if status == MOI.OPTIMAL
                if abs(r.objectives[1] + 8.214101) < 1.0e-5
                    @test r.objectives[1] ≈ -8.214101 atol=1.0e-5
                else
                    # Neither SCS nor ECOS can find the optimum.
                    @test_broken r.objectives[1] ≈ -8.214101 atol=1.0e-5
                end
            end
        else
            println(r.objectives)
            @test_broken true == false
        end

        if status == MOI.OPTIMAL
            # Don't perform these tests for MOI.ALMOST_OPTIMAL: most constraint
            # are respected, but the sum over all paths may be "slightly" off
            # (1.05, for instance).

            # This solution only uses two paths out of three, i.e. three edges out of five.
            @test r.routings[1].data == rd
            @test length(r.routings[1].demands) == ne(k) # Number of demands
            @test r.routings[1].demands[1] == d
            @test length(r.routings[1].paths) in [2, 3] # Number of paths
            @test length(r.routings[1].path_flows) == ne(k) # Number of demands
            @test length(r.routings[1].path_flows[d]) in [2, 3] # Number of paths
            @test length(r.routings[1].edge_flows) == ne(k) # Number of demands
            @test length(r.routings[1].edge_flows[d]) in [3, 4, 5] # Number of edges
            @test sum(x for x in values(r.routings[1].path_flows[d])) ≈ 1.0 atol=1.0e-3
        end
    end
end

function __testset_nouncertainty_mmf(edge_obj, type, opt, g, paths, k, d, dm)
    @testset "Objective aggregation: min-max fair" begin
        mt = ModelType(edge_obj, MinMaxFair(0.001), type, false, Automatic(), NoUncertaintyHandling(), NoUncertainty())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm, logmessage=(msg)->nothing)
        r = compute_routing(rd)

        # Similar to __testset_nouncertainty_shared, but many things differ for
        # MMF. Mostly, it's an iterative process.
        @test r !== nothing
        @test r.data == rd
        @test r.n_iter in [1, 2]
        @test r.n_matrices == 1
        @test r.n_cuts == 0
        @test r.n_columns == 0
        @test r.time_precompute_ms > 0.0
        @test r.time_solve_ms > 0.0
        @test r.time_export_ms == 0.0
        @test length(r.matrices) == 1
        @test length(r.routings) in [1, 2]
        @test length(r.objectives) in [1, 2]
        @test r.master_model !== nothing
        @test r.matrices[1] == dm

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
