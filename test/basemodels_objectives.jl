@testset "Objective terms" begin
    local_opt = ECOS.Optimizer # Support for SOCP.

    g = MetaDiGraph()
    add_vertex!(g, :name, "A")
    add_vertex!(g, :name, "B")
    add_vertex!(g, :name, "C")
    add_vertex!(g, :name, "D")
    add_edge!(g, 1, 2, :capacity, 2.0)
    add_edge!(g, 1, 3, :capacity, 2.0)
    add_edge!(g, 2, 4, :capacity, 2.0)
    add_edge!(g, 3, 4, :capacity, 2.0)
    add_edge!(g, 4, 1, :capacity, 0.001) # inedge(src), outedge(dst)

    k = SimpleDiGraph(4)
    add_edge!(k, 1, 4)
    d = Edge(1, 4)
    dm = Dict{Edge{Int}, Float64}(d => 4.0)

    mt = ModelType(Load(), MinimumTotal(), FlowFormulation(), false, CuttingPlane(), ObliviousUncertainty(), UncertainDemand())
    rd = RoutingData(g, k, local_opt, mt)
    m = Model(local_opt)
    set_silent(m)
    rm = basic_routing_model_unitary(m, rd, FlowFormulation())
    dm = Dict{Edge{Int}, Float64}(d => 4.0)

    @testset "Nonexistent function" begin
        @test_throws ErrorException objective_edge_expression(rm, TestEdgeWise(), Edge(1, 2))
    end

    @testset "Using a local traffic matrix" begin
        rd_dm = RoutingData(g, k, local_opt, mt, traffic_matrix=dm)
        m_dm = Model(local_opt)
        set_silent(m_dm)
        rm_dm = basic_routing_model_unitary(m_dm, rd_dm, FlowFormulation())
        @test objective_edge_expression(rm_dm, Load(), Edge(1, 2), dm) == objective_edge_expression(rm_dm, Load(), Edge(1, 2))
    end

    @testset "Load" begin
        @test_throws ErrorException objective_edge_expression(rm, Load(), Edge(1, 2))
        @test objective_edge_expression(rm, Load(), Edge(1, 2), dm) == total_flow_in_edge(rm, Edge(1, 2), dm) / 2.0
        @test objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 2 * rm.routing[d, Edge(1, 2)]
    end

    @testset "Kleinrock" begin
        # Needs a SOCP solver.
        cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 0.75)
        obj = objective_edge_expression(rm, KleinrockLoad(), Edge(1, 2), dm)
        @objective(rm.model, Min, obj)
        optimize!(rm.model)
        @test termination_status(rm.model) == MOI.OPTIMAL
        @test value(obj) ≈ 0.75 / 0.25
        delete(rm.model, cref)
    end

    @testset "Fortz-Thorup" begin
        # - Below 1/3.
        cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 0.3)
        obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        @objective(rm.model, Min, obj)
        optimize!(rm.model)
        @test termination_status(rm.model) == MOI.OPTIMAL
        @test value(obj) ≈ 0.3
        delete(rm.model, cref)

        # - Between 1/3 and 2/3.
        cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 0.6)
        obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        @objective(rm.model, Min, obj)
        optimize!(rm.model)
        @test termination_status(rm.model) == MOI.OPTIMAL
        @test value(obj) ≈ 1.13333333 atol=1.e-5
        delete(rm.model, cref)

        # - Between 2/3 and 9/10.
        cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 0.8)
        obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        @objective(rm.model, Min, obj)
        optimize!(rm.model)
        @test termination_status(rm.model) == MOI.OPTIMAL
        @test value(obj) ≈ 2.66666667 atol=1.e-5
        delete(rm.model, cref)

        # - Between 9/10 and 1.
        cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 0.95)
        obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        @objective(rm.model, Min, obj)
        optimize!(rm.model)
        @test termination_status(rm.model) == MOI.OPTIMAL
        @test value(obj) ≈ 7.16666667 atol=1.e-5
        delete(rm.model, cref)

        # TODO: the other parts are disabled, it seems like the values are too large for the solver to handle (!?).

        # # - Between 1 and 11/10.
        # cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 1.05)
        # obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        # @objective(rm.model, Min, obj)
        # optimize!(rm.model)
        # @test termination_status(rm.model) == MOI.OPTIMAL
        # @test value(obj) ≈ 35.6666667 atol=1.e-5
        # delete(rm.model, cref)

        # # - Over 11/10.
        # cref = @constraint(rm.model, objective_edge_expression(rm, Load(), Edge(1, 2), dm) == 1.2)
        # obj = objective_edge_expression(rm, FortzThorupLoad(), Edge(1, 2), dm)
        # @objective(rm.model, Min, obj)
        # optimize!(rm.model)
        # @test termination_status(rm.model) == MOI.OPTIMAL
        # @test value(obj) ≈ 560.666667 atol=1.e-5
        # delete(rm.model, cref)
    end
end
