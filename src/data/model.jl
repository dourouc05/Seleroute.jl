# Only for internal use.

# Indicates how `routing` should be understood:
# either as unitary flows (multiply by a demand matrix to write the capacity constraint)
# or directly flows (simple sum to write the capacity constraint)
# TODO: keep it as an enumaration or go to an abstract type?
@enum(FlowScaling, UnitaryFlows, UnscaledFlows)

mutable struct RoutingModel # TODO: use Unicode characters for Greek letters?
    data::RoutingData
    model::Model
    scaled_flows::FlowScaling

    ## Variables.
    mu # Either a maximum load or an oblivious ratio.
    tau # MMF objective (i.e. a threshold).
    routing # Main variable when optimising a routing.
    demand # When optimising for a traffic matrix (like oblivious subproblems).

    # For dual reformulations (robust and oblivious). Meaning heavily depends
    # on the actual model. These are not the dual values associated to the
    # constraints of the problem, but dual values coming from a reformulation.
    dual
    dual_alpha
    dual_beta
    dual_gamma
    dual_delta

    ## Constraints.
    constraints_capacity::Dict{Edge{Int}, Any} # Edge -> constraint ref
    constraints_mmf # Edge -> constraint ref (AbstractMatrix or Dict)

    # Flow-based.
    constraints_source_in::Dict{Edge{Int}, Any} # Demand -> constraint ref
    constraints_source_out::Dict{Edge{Int}, Any} # Demand -> constraint ref
    constraints_target_in::Dict{Edge{Int}, Any} # Demand -> constraint ref
    constraints_target_out::Dict{Edge{Int}, Any} # Demand -> constraint ref
    constraints_balance::Dict{Edge{Int}, Dict{Int, Any}} # Demand -> node -> constraint ref

    # Path-based.
    constraints_convexity # Demand -> constraint ref
    constraints_matrices::Dict{Dict{Edge{Int}, Float64}, Dict{Edge{Int}, Any}} # Matrix::Dict{Edge{Int}, Float64} -> edge -> constraint ref
    constraints_uncertainty_set::Dict{Edge{Int}, Dict{Edge{Int}, ConstraintRef}} # Edge -> demand -> constraint ref

    function RoutingModel(data::RoutingData, model::Model, scaled_flows::FlowScaling, routing;
        demand=nothing, mu=nothing, tau=nothing,
        dual=nothing, dual_alpha=nothing, dual_beta=nothing, dual_gamma=nothing, dual_delta=nothing,
        constraints_capacity::Dict{Edge{Int}, Any}=Dict{Edge{Int}, Any}(),
        constraints_mmf=nothing,
        constraints_source_in::Dict{Edge{Int}, Any}=Dict{Edge{Int}, Any}(), constraints_source_out::Dict{Edge{Int}, Any}=Dict{Edge{Int}, Any}(),
        constraints_target_in::Dict{Edge{Int}, Any}=Dict{Edge{Int}, Any}(), constraints_target_out::Dict{Edge{Int}, Any}=Dict{Edge{Int}, Any}(),
        constraints_balance::Dict{Edge{Int}, Dict{Int, Any}}=Dict{Edge{Int}, Dict{Int, Any}}(),
        constraints_convexity=nothing, constraints_matrices::Dict{Dict{Edge{Int}, Float64}, Dict{Edge{Int}, Any}}=Dict{Dict{Edge{Int}, Float64}, Dict{Edge{Int}, Any}}(),
        constraints_uncertainty_set::Dict{Edge{Int}, Dict{Edge{Int}, ConstraintRef}}=Dict{Edge{Int}, Dict{Edge{Int}, ConstraintRef}}()
    )
        @assert routing !== nothing

        return new(data, model, scaled_flows, mu, tau, routing, demand,
            dual, dual_alpha, dual_beta, dual_gamma, dual_delta,
            constraints_capacity, constraints_mmf,
            constraints_source_in, constraints_source_out, constraints_target_in, constraints_target_out, constraints_balance,
            constraints_convexity, constraints_matrices, constraints_uncertainty_set
        )
    end
end

function add_path!(rm::RoutingModel, demand::Edge, path::Vector{Edge})
    # TODO: propose to directly call `add_routing_var!`?
    return add_path!(rm.data, demand, path)
end

function add_routing_var!(rm::RoutingModel, demand::Edge, path_id::Int;
                          constraint_capacity::Bool=false)
    # Preconditions: the path already exists, this is a unitary-flow model,
    # this is a path-based formulation, this model has a convexity constraint.
    rd = rm.data
    @assert path_id in rd.demand_to_path_ids[demand]

    if ! (path_id in rd.demand_to_path_ids[demand])
        error("The path ID $(path_id) has not been added to the model.")
    end
    if rd.model_type.type != PathFormulation()
        error("This model is not a path-based formulation, but rather a $(rd.model_type). Impossible to add a new path in it.")
    end
    if length(rm.constraints_convexity) != n_demands(rd)
        error("This model does not look like it has a convexity constraint.")
    end

    # TODO: check for integer flows.
    # TODO: if constraint_capacity, ensure no ambiguity on traffic/capacity matrix.

    # Create the variable.
    if rm.scaled_flows == UnitaryFlows
        rm.routing[demand, path_id] = @variable(rm.model, lower_bound=0, upper_bound=1)
    else
        rm.routing[demand, path_id] = @variable(rm.model, lower_bound=0)
    end
    set_name(rm.routing[demand, path_id], "routing[$demand, $path_id]")

    # Add it into the convexity constraint.
    set_normalized_coefficient(rm.constraints_convexity[demand], rm.routing[demand, path_id], 1)
    # TODO: is it always 1? So far, yes...

    # Add it into the capacity constraint, if asked for.
    if constraint_capacity
        for e in rd.paths_edges[path_id]
            if rm.scaled_flows == UnitaryFlows
                coeff = rd.traffic_matrix[demand]
            elseif rm.scaled_flows == UnscaledFlows
                coeff = 1
            else
                error("Not implemented: $(rm.scaled_flows)")
            end

            set_normalized_coefficient(rm.constraints_capacity[e], rm.routing[demand, path_id], coeff)
        end
    end

    # TODO: other generic constraints (i.e. could be reused somewhere else)?
end

# function copy_model(rm::RoutingModel)
#     m2, refmap = copy_model(rm.model)
#     # TODO: translate more things? This is the bare minimum, and probably not even working in all needed cases...
#     return RoutingModel(rm.data, m2, refmap[rm.mu], refmap[rm.routing])
# end

# Channel the accessors through.
graph(rm::RoutingModel) = graph(rm.data)
edges(rm::RoutingModel) = edges(rm.data)
vertices(rm::RoutingModel) = vertices(rm.data)
demands(rm::RoutingModel) = edges(rm.data)

n_nodes(rm::RoutingModel) = n_nodes(rm.data)
n_edges(rm::RoutingModel) = n_edges(rm.data)
n_demands(rm::RoutingModel) = n_demands(rm.data)
n_paths(rm::RoutingModel) = n_paths(rm.data)

capacity(rm::RoutingModel, e::Edge{Int}) = capacity(rm.data, e)
