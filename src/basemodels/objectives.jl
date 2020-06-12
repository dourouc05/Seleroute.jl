function objective_edge_expression(::RoutingModel, edge_obj::EdgeWiseObjectiveFunction, ::Edge, ::Dict{Edge{Int}, Float64})
    error("Edge-wise objective function not implemented: $edge_obj")
end

function objective_edge_expression(rm::RoutingModel, edge_obj::EdgeWiseObjectiveFunction, e::Edge)
    if length(rm.data.traffic_matrix) == 0
        error("Creating an edge-wise objective term with an empty default traffic matrix. " *
              "Set one in the corresponding RoutingData object or use the `dm` argument.")
    end

    return objective_edge_expression(rm, edge_obj, e, rm.data.traffic_matrix)
end

function objective_edge_expression(rm::RoutingModel, ::Load, e::Edge, dm::Dict{Edge{Int}, Float64})
    return total_flow_in_edge(rm, e, dm) / get_prop(rm.data.g, e, :capacity)
end

function objective_edge_expression(rm::RoutingModel, ::KleinrockLoad, e::Edge, dm::Dict{Edge{Int}, Float64})
    # Model the function load / (1 - load). The constraint is a rotated SOCP:
    #     load / (1 - load) ≤ kl    ⟺    (√2)² ≤ 2 (1 - load) (1 + kl)
    kleinrock = @variable(rm.model, base_name="kleinrock[$e]", lower_bound=0)

    load_e = objective_edge_expression(rm, Load(), e, dm)
    c = @constraint(rm.model, [1 - load_e, 1 + kleinrock, sqrt(2)] in RotatedSecondOrderCone())
    set_name(c, "kleinrock[$(e)]")

    return kleinrock
end

function objective_edge_expression(rm::RoutingModel, ::FortzThorupLoad, e::Edge, dm::Dict{Edge{Int}, Float64})
    fortzthorup = @variable(rm.model, base_name="fortzthorup[$e]", lower_bound=0)
    load_e = objective_edge_expression(rm, Load(), e, dm)

    c1 = @constraint(rm.model, fortzthorup >= load_e)
    set_name(c1, "fortzthorup_1[$(e)]")

    c2 = @constraint(rm.model, fortzthorup >= 3 * load_e - 2 / 3)
    set_name(c2, "fortzthorup_2[$(e)]")

    c3 = @constraint(rm.model, fortzthorup >= 10 * load_e - 16 / 3)
    set_name(c3, "fortzthorup_3[$(e)]")

    c4 = @constraint(rm.model, fortzthorup >= 70 * load_e - 178 / 3)
    set_name(c4, "fortzthorup_4[$(e)]")

    c5 = @constraint(rm.model, fortzthorup >= 500 * load_e - 1486 / 3)
    set_name(c5, "fortzthorup_5[$(e)]")

    c6 = @constraint(rm.model, fortzthorup >= 5000 * load_e - 16318 / 3)
    set_name(c6, "fortzthorup_6[$(e)]")

    return fortzthorup
end

function objective_edge_expression(rm::RoutingModel, edge_obj::AlphaFairness, e::Edge, dm::Dict{Edge{Int}, Float64})
    # Model the function load^α / (1 - α). Depending on the value of α,
    # this constraint has very different models. They are all reformulation
    # of the inequality load^α / (1 - α) ≥ af.
    α = edge_obj.α
    @assert edge_obj.α >= 0

    # Α-fairness can be quite low, but bound it below to avoid unboundedness in the tests.
    alphafairness = @variable(rm.model, base_name="alphafairness[$e]", lower_bound=-2.0e4)
    load_e = objective_edge_expression(rm, Load(), e, dm)

    c = if α == 0.5
        # This is a very specific case where the power cone is not really
        # needed, two second-order cones are sufficient. These cone are more
        # widely tolerated.
        #     af ≤ ½ √load
        #   ⟺ ϕ² ≤ l          ∧    af ≤ ½ √(ϕ ϕ)
        #   ⟺ ϕ² ≤ l × 1/2    ∧    af² ≤ (ϕ/√2) (ϕ/√2)
        ϕ = @variable(rm.model, base_name="alphafairness_phi[$e]", lower_bound=0)

        # ϕ² ≤ l
        c = @constraint(rm.model, [ϕ, ϕ, load_e] in RotatedSecondOrderCone())
        set_name(c, "alphafairness_phi[$(e)]")

        # af ≤ √(ϕ ϕ)
        @constraint(rm.model, [ϕ/2, ϕ/2, alphafairness] in RotatedSecondOrderCone())
    elseif α == 1.0
        # The function becomes a logarithm:
        #     af ≤ log(load)
        #     1 × exp(af / 1) ≤ load
        #     (af, 1, l) ∈ EXP
        @constraint(rm.model, [alphafairness, 1, load_e] in MOI.ExponentialCone())
    elseif α < 1.0
        # Generic case: 0.0 < α < 1.0 (also works for α = ½).
        #     af ≤ load^(1-α) / (1 - α),     with α < 1, i.e. 1 - α > 0
        #   ⟺ af ≤ load^(1-α) × 1^α / (1 - α)
        #   ⟺ (1 - α) × af ≤ load^(1-α) × 1^α
        @constraint(rm.model, [(1 - α) * load_e, 1, alphafairness] in MOI.PowerCone(1.0 - α))
    else
        @assert α > 1.0
        # Last case: α > 1.0.
        #     af ≤ load^(1-α) / (1 - α),     with α > 1, i.e. 1-α < 0
        #   ⟺ 1 ≤ load^(1-α) × t^-1 / (1 - α)
        #   ⟺ 1 - α ≤ load^(1-α) × t^-1
        #   ⟺ (1 - α)^(-α)  ≤ load^(1-α)/(-α) × t^-1/(-α)
        #   ⟺ 1 / (1 - α)^α ≤ load^(α-1)/α × 1^(1/α)
        @constraint(rm.model, [1.0 / (1 - α)^α, load_e, alphafairness] in MOI.PowerCone((α - 1.0) / α))
    end

    set_name(c, "alphafairness[$(e)]")
    return alphafairness
end
