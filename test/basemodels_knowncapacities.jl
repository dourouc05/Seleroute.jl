@testset "Base models: known capacities" begin
    opt = ECOS.Optimizer

    g = MetaDiGraph()
    add_vertex!(g, :name, "A")
    add_vertex!(g, :name, "B")
    add_vertex!(g, :name, "C")
    add_vertex!(g, :name, "D")
    add_edge!(g, 1, 2, :capacity, 2.0)
    add_edge!(g, 1, 3, :capacity, 2.0)
    add_edge!(g, 2, 4, :capacity, 2.0)
    add_edge!(g, 3, 4, :capacity, 2.0)

    k = SimpleDiGraph(4)
    add_edge!(k, 1, 4)
    d = Edge(1, 4)
    dm = Dict{Edge{Int}, Float64}(d => 4.0)

    @testset "Base case" begin
        m = Model(opt)
        mt = ModelType(Load(), MinimumTotal(), FlowFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        rd = RoutingData(g, k, opt, mt)
        rm = basic_routing_model_unitary(m, rd, FlowFormulation())

        @test_throws ErrorException basic_routing_model_unitary(m, rd, 42)
        @test_throws ErrorException basic_routing_model_unscaled(m, rd, 42)

        @test_throws ErrorException total_flow_in_edge(rm, Edge(1, 2), 42)
        @test_throws ErrorException total_flow_in_edge(rm, Edge(1, 2), dm, 42)
    end

    @testset "Flow" begin
        # Only useful parameter: FlowFormulation.
        mt = ModelType(Load(), MinimumTotal(), FlowFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)

        function __check_feasible_knowncapa_flow(rm::RoutingModel)
            @test rm.routing !== nothing
            @test rm.constraints_source_in !== nothing && length(rm.constraints_source_in) >= 0
            @test rm.constraints_source_out !== nothing && length(rm.constraints_source_out) > 0
            @test rm.constraints_target_in !== nothing && length(rm.constraints_target_in) > 0
            @test rm.constraints_target_out !== nothing && length(rm.constraints_target_out) >= 0
            @test rm.constraints_balance !== nothing && length(rm.constraints_balance) > 0
            @test rm.constraints_convexity === nothing || length(rm.constraints_convexity) == 0

            # Check that the model has a feasible solution.
            set_silent(rm.model)
            optimize!(rm.model)
            @test termination_status(rm.model) == MOI.OPTIMAL

            @test value(rm.routing[d, Edge(1, 2)]) == value(rm.routing[d, Edge(2, 4)])
            @test value(rm.routing[d, Edge(1, 3)]) == value(rm.routing[d, Edge(3, 4)])

            # Check that the model correctly enforces the connectivity.
            for e in [Edge(1, 2), Edge(2, 4), Edge(1, 3), Edge(3, 4)]
                @objective(rm.model, Max, rm.routing[d, e])
                optimize!(rm.model)
                @test termination_status(rm.model) == MOI.OPTIMAL

                @test value(rm.routing[d, Edge(1, 2)]) ≈ value(rm.routing[d, Edge(2, 4)]) atol=1.0e-5
                @test value(rm.routing[d, Edge(1, 3)]) ≈ value(rm.routing[d, Edge(3, 4)]) atol=1.0e-5
            end
        end

        function __check_feasible_knowncapa_withcapa_flow(rm::RoutingModel)
            # Capacity helpers.
            @test rm.constraints_capacity === nothing || length(rm.constraints_capacity) == 0
            capacity_constraints(rm, dm)
            @test rm.constraints_capacity !== nothing && length(rm.constraints_capacity) > 0

            @objective(rm.model, Max, rm.routing[d, Edge(1, 2)])
            optimize!(rm.model)
            @test termination_status(rm.model) == MOI.OPTIMAL

            @test value(rm.routing[d, Edge(1, 2)]) ≈ value(rm.routing[d, Edge(2, 4)]) atol=1.0e-5
            @test value(rm.routing[d, Edge(1, 3)]) ≈ value(rm.routing[d, Edge(3, 4)]) atol=1.0e-5

            @test value(total_flow_in_edge(rm, Edge(1, 2), dm)) ≈ 2
            @test value(total_flow_in_edge(rm, Edge(1, 3), dm)) ≈ 2

            if rm.scaled_flows == UnitaryFlows
                @test_throws AssertionError value(total_flow_in_edge(rm, Edge(1, 2))) ≈ 2
            else
                @test value(total_flow_in_edge(rm, Edge(1, 2))) ≈ 2
            end

            # Incompatible with other capacity constraints.
            @test_throws AssertionError mu_capacity_constraints(rm, dm)
        end

        @testset "Unitary flows" begin
            m = Model(opt)
            rm = basic_routing_model_unitary(m, rd, FlowFormulation())

            @test rm.data == rd
            @test rm.model == m
            @test rm.scaled_flows == UnitaryFlows

            __check_feasible_knowncapa_flow(rm)
            __check_feasible_knowncapa_withcapa_flow(rm)

            # Helpers.
            @test total_flow_in_edge(rm, Edge(1, 2), dm) == 4 * rm.routing[d, Edge(1, 2)]
        end

        @testset "Unscaled flows" begin
            m = Model(opt)
            rm = basic_routing_model_unscaled(m, rd, dm, FlowFormulation())

            @test rm.data == rd
            @test rm.model == m
            @test rm.scaled_flows == UnscaledFlows

            __check_feasible_knowncapa_flow(rm)
            __check_feasible_knowncapa_withcapa_flow(rm)

            # Helpers.
            @test total_flow_in_edge(rm, Edge(1, 2), dm) == rm.routing[d, Edge(1, 2)]
        end

        @testset "µ-capacited unitary flows" begin
            m = Model(opt)
            rm = basic_routing_model_unitary(m, rd, FlowFormulation())
            mu_capacity_constraints(rm, dm)
            @objective(m, Min, rm.mu)

            __check_feasible_knowncapa_flow(rm)

            # Helpers.
            @test total_flow_in_edge(rm, Edge(1, 2), dm) == 4 * rm.routing[d, Edge(1, 2)]

            # Incompatible with other capacity constraints.
            @test_throws AssertionError capacity_constraints(rm, dm)
            @test_throws AssertionError mu_capacity_constraints(rm)
        end
    end

    @testset "Path" begin
        # Only useful parameter: PathFormulation.
        mt = ModelType(Load(), MinimumTotal(), PathFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
        rd = RoutingData(g, k, opt, mt, traffic_matrix=dm)

        function __check_feasible_knowncapa_path(rm::RoutingModel)
            @test rm.routing !== nothing
            @test rm.constraints_source_in === nothing || length(rm.constraints_source_in) == 0
            @test rm.constraints_source_out === nothing || length(rm.constraints_source_out) == 0
            @test rm.constraints_target_in === nothing || length(rm.constraints_target_in) == 0
            @test rm.constraints_target_out === nothing || length(rm.constraints_target_out) == 0
            @test rm.constraints_balance === nothing || length(rm.constraints_balance) == 0
            @test rm.constraints_convexity !== nothing && length(rm.constraints_convexity) > 0

            # Check that the model has a feasible solution.
            set_silent(rm.model)
            optimize!(rm.model)
            @test termination_status(rm.model) == MOI.OPTIMAL

            if rm.scaled_flows == UnitaryFlows
                @test 0 <= value(rm.routing[d, 1]) <= 1
                @test 0 <= value(rm.routing[d, 2]) <= 1
            else
                @test rm.scaled_flows == UnscaledFlows
                @test value(rm.routing[d, 1]) >= 0
                @test value(rm.routing[d, 2]) >= 0
            end

            # Check that the model correctly enforces the connectivity.
            for p in [1, 2]
                @objective(rm.model, Max, rm.routing[d, p])
                optimize!(rm.model)
                @test termination_status(rm.model) == MOI.OPTIMAL

                @test value(rm.routing[d, 1]) + value(rm.routing[d, 2]) ≈ ((rm.scaled_flows == UnitaryFlows) ? 1 : 4)
            end

            # Capacity helpers.
            @test rm.constraints_capacity === nothing || length(rm.constraints_capacity) == 0
            capacity_constraints(rm, dm)
            @test rm.constraints_capacity !== nothing && length(rm.constraints_capacity) > 0

            @objective(rm.model, Max, rm.routing[d, 1])
            optimize!(rm.model)
            @test termination_status(rm.model) == MOI.OPTIMAL

            @test value(total_flow_in_edge(rm, Edge(1, 2), dm)) ≈ 2
            @test value(total_flow_in_edge(rm, Edge(1, 3), dm)) ≈ 2

            if rm.scaled_flows == UnitaryFlows
                @test_throws AssertionError value(total_flow_in_edge(rm, Edge(1, 2))) ≈ 2
            else
                @test value(total_flow_in_edge(rm, Edge(1, 2))) ≈ 2
            end

            # Helpers.
            tf12 = total_flow_in_edge(rm, Edge(1, 2), dm)
            if rm.scaled_flows == UnitaryFlows
                @test tf12 == 4 * rm.routing[d, 1]
            else
                @test rm.scaled_flows == UnscaledFlows
                @test tf12 == rm.routing[d, 1]
            end
        end

        @testset "Unitary flows" begin
            m = Model(opt)
            rm = basic_routing_model_unitary(m, rd, PathFormulation())

            @test rm.data == rd
            @test rm.model == m
            @test rm.scaled_flows == UnitaryFlows

            __check_feasible_knowncapa_path(rm)
        end

        @testset "Unscaled flows" begin
            m = Model(opt)
            rm = basic_routing_model_unscaled(m, rd, dm, PathFormulation())

            @test rm.data == rd
            @test rm.model == m
            @test rm.scaled_flows == UnscaledFlows

            __check_feasible_knowncapa_path(rm)
        end
    end
end
