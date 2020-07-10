# Only for internal use.

# Indicates how `routing` should be understood:
# either as unitary flows (multiply by a demand matrix to write the capacity constraint)
# or directly flows (simple sum to write the capacity constraint)
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

# function copy_model(rm::RoutingModel)
#     m2, refmap = copy_model(rm.model)
#     # TODO: translate more things? This is the bare minimum, and probably not even working in all needed cases...
#     return RoutingModel(rm.data, m2, refmap[rm.mu], refmap[rm.routing])
# end
