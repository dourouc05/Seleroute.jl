# Only for internal use.

# Indicates how `routing` should be understood:
# either as unitary flows (multiply by a demand matrix to write the capacity constraint)
# or directly flows (simple sum to write the capacity constraint)
@enum(FlowScaling, UnitaryFlows, UnscaledFlows)

mutable struct RoutingModel
    data::RoutingData
    model::Model
    scaled_flows::FlowScaling

    ## Variables.
    mu # Usually, a VariableRef; only for oblivious; not present for subproblems.
    routing

    # For subproblems.
    demand

    # For dual reformulations (robust and oblivious).
    dual
    dual_alpha
    dual_beta
    dual_gamma
    dual_delta

    ## Constraints.
    constraints_capacity::Dict{Edge, Any} # Edge -> constraint ref

    # Flow-based.
    constraints_source_in::Dict{Edge, Any} # Demand -> constraint ref
    constraints_source_out::Dict{Edge, Any} # Demand -> constraint ref
    constraints_target_in::Dict{Edge, Any} # Demand -> constraint ref
    constraints_target_out::Dict{Edge, Any} # Demand -> constraint ref
    constraints_balance::Dict{Edge, Dict{Int, Any}} # Demand -> node -> constraint ref

    # Path-based.
    constraints_convexity
    constraints_matrices::Dict{Dict{Edge, Float64}, Dict{Edge, Any}} # Matrix::Dict{Edge, Float64} -> edge -> constraint ref
    constraints_uncertainty_set::Dict{Edge, Dict{Edge, ConstraintRef}} # Edge -> demand -> constraint ref

    function RoutingModel(data::RoutingData, model::Model, scaled_flows::FlowScaling, mu, routing;
        demand=nothing,
        dual=nothing, dual_alpha=nothing, dual_beta=nothing, dual_gamma=nothing, dual_delta=nothing,
        constraints_capacity::Dict{Edge, Any}=Dict{Edge, Any}(),
        constraints_source_in::Dict{Edge, Any}=Dict{Edge, Any}(), constraints_source_out::Dict{Edge, Any}=Dict{Edge, Any}(),
        constraints_target_in::Dict{Edge, Any}=Dict{Edge, Any}(), constraints_target_out::Dict{Edge, Any}=Dict{Edge, Any}(),
        constraints_balance::Dict{Edge, Dict{Int, Any}}=Dict{Edge, Dict{Int, Any}}(),
        constraints_convexity=nothing, constraints_matrices::Dict{Dict{Edge, Float64}, Dict{Edge, Any}}=Dict{Dict{Edge, Float64}, Dict{Edge, Any}}(),
        constraints_uncertainty_set::Dict{Edge, Dict{Edge, ConstraintRef}}=Dict{Edge, Dict{Edge, ConstraintRef}}()
    )
        @assert routing !== nothing

        return new(data, model, scaled_flows, mu, routing,
            demand,
            dual, dual_alpha, dual_beta, dual_gamma, dual_delta,
            constraints_capacity,
            constraints_source_in, constraints_source_out, constraints_target_in, constraints_target_out, constraints_balance,
            constraints_convexity, constraints_matrices, constraints_uncertainty_set
        )
    end
end

# function copy_model(rm::RoutingModel)
#     m2, refmap = copy_model(rm.model)
#     # TODO: translate more things? This is the bare minimum, and probably not even working in all needed cases...
#     return RoutingModel(rm.data, m2, refmap[rm.mu], refmap[rm.routing])
# end