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
