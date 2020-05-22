# TODO: type for dm::DemandMatrix? or TrafficMatrix?

basic_routing_model_unitary(m::Model, rd::RoutingData) =
    basic_routing_model_unitary(m, rd, rd.model_type.type)
basic_routing_model_unitary(m::Model, rd::RoutingData, ::Any...) =
    error("Not implemented: $(rd.model_type)")
basic_routing_model_unscaled(m::Model, rd::RoutingData, ::Any...) =
    error("Not implemented: $(rd.model_type)")

function total_flow_in_edge(rm::RoutingModel, e::Edge)
    @assert rm.scaled_flows == UnscaledFlows
    return total_flow_in_edge(rm, e, rm.data.model_type.type, Val(UnscaledFlows))
end

function total_flow_in_edge(rm::RoutingModel, e::Edge, dm::Union{Dict{Edge, Float64}, Nothing})
    # Switch on the flow scale: don't pass a traffic matrix when it's not needed.
    if rm.scaled_flows == UnitaryFlows
        if dm === nothing
            dm = rm.data.traffic_matrix
        end
        @assert dm !== nothing
        return total_flow_in_edge(rm, e, dm, rm.data.model_type.type, Val(UnitaryFlows))
    elseif rm.scaled_flows == UnscaledFlows
        # dm not needed.
        return total_flow_in_edge(rm, e, rm.data.model_type.type, Val(UnscaledFlows))
    else
        error("Not implemented: $(rm.data.model_type)")
    end
end
total_flow_in_edge(rm::RoutingModel, edge::Edge, ::Any...) =
    error("Not implemented: $(rm.data.model_type)")
total_flow_in_edge(rm::RoutingModel, edge::Edge, dm::Union{Dict{Edge, Float64}, Nothing}, ::Any...) =
    error("Not implemented: $(rm.data.model_type)")

# Interpretation of the dm argument: demand -> value

function capacity_constraints(rm::RoutingModel, dm::Union{Dict{Edge, Float64}, Nothing}=nothing)
    # Capacity constraints, one per edge.
    @assert isempty(rm.constraints_capacity)
    rm.constraints_capacity = Dict{Edge, Any}()
    for e in edges(rm.data)
        flow = total_flow_in_edge(rm, e, dm)
        if ! iszero(flow)
            rm.constraints_capacity[e] = @constraint(rm.model, flow <= get_prop(rm.data.g, e, :capacity))
        end
    end
end

function mu_capacity_constraints(rm::RoutingModel, dm::Union{Dict{Edge, Float64}, Nothing}=nothing)
    # Difference with capacity_constraints: <= mu * capacity, instead of simply capacity.

    # Create the mu variable if it does not exist yet.
    if rm.mu === nothing
        rm.mu = @variable(rm.model, lower_bound=0.0)
        set_name(rm.mu, "mu")
    end

    # Capacity constraints, one per edge.
    @assert isempty(rm.constraints_capacity)
    rm.constraints_capacity = Dict{Edge, Any}()
    for e in edges(rm.data)
        flow = total_flow_in_edge(rm, e, dm)
        if ! iszero(flow)
            rm.constraints_capacity[e] = @constraint(rm.model, flow <= rm.mu * get_prop(rm.data.g, e, :capacity))
        end
    end
end
