"""
The edge-wise objective function to optimise. These values are natively supported:

* `Load`: the load of each edge is considered.
* `KleinrockLoad`: Kleinrock function is applied on the load.
* `FortzThorupLoad`: Fortz-Thorup linearisation of Kleinrock function is applied on the load.

Kleinrock function is defined as:

``K(x) = \\frac{x}{1-x}``
"""
abstract type EdgeWiseObjectiveFunction end

supports_min(::EdgeWiseObjectiveFunction) = false
supports_max(::EdgeWiseObjectiveFunction) = false

struct Load <: EdgeWiseObjectiveFunction end
struct FortzThorupLoad <: EdgeWiseObjectiveFunction end

struct KleinrockLoad <: EdgeWiseObjectiveFunction
    use_nonconvex_bilinear_formulation_for_equality::Bool

    KleinrockLoad(b::Bool=false) = new(b)

    # Solver is supposed to be able to handle bilinear constraints!
    # Outside generic global/MINLP solvers, there is just Gurobi to handle this:
    #     optimizer_with_attributes(Gurobi.Optimizer, "NonConvex" => 2)
end

supports_min(::Union{Load, KleinrockLoad, FortzThorupLoad}) = true
supports_max(::Load) = true
supports_max(k::KleinrockLoad) = k.use_nonconvex_bilinear_formulation_for_equality
supports_max(::FortzThorupLoad) = false

struct AlphaFairness <: EdgeWiseObjectiveFunction
    α::Float64
    force_power_cone::Bool # When there is a choice between a SOCP-based and a power-cone-based formulation, use the latter.

    AlphaFairness() = new(1.0, false)
    function AlphaFairness(α::Float64, force_power_cone::Bool=false)
        if force_power_cone
            if α == 0.0
                @warn "The power-cone formulation is forced to be used, " *
                      "but it is not available for α = $(α). " *
                      "The parameter `force_power_cone` is therefore ignored."
            elseif ! (α in [0.5, 1.5, 2.0]) # Easily SOCP-representable.
                @warn "The power-cone formulation is forced to be used, " *
                      "but it is the only available one for α = $(α). " *
                      "The parameter `force_power_cone` is therefore ignored."
            end
        end
        return new(α, force_power_cone)
    end
end

supports_min(af::AlphaFairness) = af.α == 0.0 # Only for LP model.
supports_max(::AlphaFairness) = true

"""
The aggregation-function objective to optimise, giving one objective function aggregating
all the components for each edge (indicated by `EdgeWiseObjectiveFunction`). These values are natively supported:

* `MinimumTotal`: the total considered value is minimised.
* `MinimumMaximum`: the maximum considered value is minimised.
* `MinMaxFair`: a min-max-fair solution is computed with respect to the considered value.
  By definition, this means that each user has their own value; not a single user can reduce
  this value without others seeing an increase.

In theory, a max-min fair solution is kept when any nondecreasing function is applied on the criteria.
It becomes a min-max fair solution if the function is nonincreasing. In practice, it means that
using `MinMaxFair` with `KleinrockLoad` or `Load` should give the same solution. The solver will
not forbid such combinations, though.
"""
abstract type AggregationObjectiveFunction end

abstract type MinimisingAggregationObjectiveFunction <: AggregationObjectiveFunction end
abstract type MaximisingAggregationObjectiveFunction <: AggregationObjectiveFunction end

struct MinimumTotal <: MinimisingAggregationObjectiveFunction end
struct MinimumMaximum <: MinimisingAggregationObjectiveFunction end
struct MinMaxFair <: MinimisingAggregationObjectiveFunction
    ε::Float64 # Numerical accuracy when adding MMF constraints.

    MinMaxFair(ε::Float64) = new(ε)
    MinMaxFair() = new(1.0e-5)
end

struct MaximumTotal <: MaximisingAggregationObjectiveFunction end
struct MaximumMinimum <: MaximisingAggregationObjectiveFunction end
struct MaxMinFair <: MaximisingAggregationObjectiveFunction
    ε::Float64 # Numerical accuracy when adding MMF constraints.

    MaxMinFair(ε::Float64) = new(ε)
    MaxMinFair() = new(1.0e-5)
end

"""
The type of formulation that is used to solve the routing problem. This package natively supports two main families of formulations:

* `FlowFormulation`: each edge is explicitly present in the model
* `PathFormulation`: the model uses path variables. The path-based formulations can be solved in two ways:
  * precomputing paths: only the set of precomputed paths is considered; if not enough paths are precomputed,
    the solution is only approximate, and no approximation factor is guaranteed
  * column generation: new paths are added on the fly, when they might improve the solution
"""
abstract type FormulationType end

struct FlowFormulation <: FormulationType end
struct PathFormulation <: FormulationType end

"""
For robust and oblivious uncertainties, this package natively offers multiple algorithm choices:

* `Automatic`: let the package decide which algorith to use
* `CuttingPlane`: use an iterative cutting-plane algorithm (solves many small problems)
* `DualReformulation`: use a dual-reformulation of the uncertainty set (solves one large problem)

For other types of uncertainty, use the value `Automatic`.
"""
abstract type AlgorithmChoice end

struct Automatic <: AlgorithmChoice end
struct CuttingPlane <: AlgorithmChoice end
struct DualReformulation <: AlgorithmChoice end
# TODO: Different algorithms for MMF?
# TODO: mix between algorithms.

"""
The way uncertainty is implemented. These values are natively supported:

* `NoUncertaintyHandling`: the uncertainty is not taken into account, only nominal values are used
* `StochasticUncertainty`: the uncertainty is implemented through stochastic optimisation,
  i.e. with several (potentially weighted) values of the unknown parameters
* `RobustUncertainty`: the uncertainty is implemented through robust optimisation,
  i.e. the unknown parameters belong to a given (bounded) uncertainty set
* `ObliviousUncertainty`: the uncertainty is implemented through oblivious optimisation,
  i.e. the unknown parameters belong to an unbouded uncertainty set
"""
abstract type UncertaintyHandling end

struct NoUncertaintyHandling <: UncertaintyHandling end
struct StochasticUncertainty <: UncertaintyHandling end
struct RobustUncertainty <: UncertaintyHandling end
struct ObliviousUncertainty <: UncertaintyHandling end

"""
The parameters of the problem that can see uncertainty. These values are natively supported:

* `NoUncertainty`: there is no uncertainty in the problem
* `UncertainDemand`: the demand to route are not fully known, but the topology has no uncertainty
* `UncertainCapacity`: the links have unknown capacities, but the demands to route are fully known
"""
abstract type UncertainParameters end

struct NoUncertainty <: UncertainParameters end
struct UncertainDemand <: UncertainParameters end
struct UncertainCapacity <: UncertainParameters end

"""
Parameterising the way a routing-optimisation problem must be solved. Seven parameters must be chosen (independently
of each other):

* an edge-wise objective function, which assigns a value for each edge, of type `EdgeWiseObjectiveFunction`
* an aggregation technique to build the complete objective function from the edge-wise pieces, of type `AggregationObjectiveFunction`
* a formulation type, of type `FormulationType` (for path-based formulations, column generation can be separately enabled)
* whether column generation is enabled, only for path-based formulations
* a way to heandle the uncertainty, of type `UncertaintyHandling`
* a precise algorithm to solve the instance, of type `AlgorithmChoice`
* a definition of which parameters can be afflicted by uncertainty, of type `UncertainParameters`

The exact algorithm used to solve the instance is based on those seven parameters.
"""
struct ModelType
    # TODO: integer vs. continuous flows?
    edge_obj::EdgeWiseObjectiveFunction
    agg_obj::AggregationObjectiveFunction
    type::FormulationType
    cg::Bool
    algo::AlgorithmChoice
    unc::UncertaintyHandling
    uncparams::UncertainParameters

    function ModelType(edge_obj::EdgeWiseObjectiveFunction,
                       agg_obj::AggregationObjectiveFunction,
                       type::FormulationType=FlowFormulation(),
                       cg::Bool=false,
                       algo::AlgorithmChoice=Automatic(),
                       unc::UncertaintyHandling=NoUncertaintyHandling(),
                       uncparams::UncertainParameters=NoUncertainty())
        is_obj_load = edge_obj in [Load(), KleinrockLoad(), FortzThorupLoad()]
        if is_obj_load && uncparams == UncertainCapacity()
            error("Load-related edge-wise objective functions like $(edge_obj) are not supposed to be used with capacity uncertainty.")
        end

        if type == FlowFormulation() && cg
            error("Column generation is not defined for flow formulations")
        end

        if (unc != ObliviousUncertainty() && unc != RobustUncertainty()) && algo != Automatic()
            # TODO: MMF?
            error("Different algorithms are only available for robust- and oblivious-uncertainty models.")
        end
        if (unc == ObliviousUncertainty() || unc == RobustUncertainty()) && algo == Automatic()
            algo = DualReformulation() # Usually the most efficient one. Could implement heuristics based on numerical experiments.
        end

        if (unc == NoUncertaintyHandling() && uncparams != NoUncertainty()) || (unc == NoUncertaintyHandling() && uncparams != NoUncertainty())
            text = "When there is no uncertainty, the parameters UncertaintyHandling"
            if unc != NoUncertaintyHandling() && uncparams == NoUncertainty()
                text *= " ($unc != $(NoUncertaintyHandling()))"
            end
            text *= " and UncertainParameters"
            if unc == NoUncertaintyHandling() && uncparams != NoUncertainty()
                text *= " ($uncparams != $(NoUncertainty()))"
            end
            text *= " must match."
            error(text)
        end

        if uncparams == UncertainCapacity()
            error("No model implemented uncertainty on link capacity")
        end

        # Check that the parameter detection worked.
        if unc == ObliviousUncertainty() || unc == RobustUncertainty()
            @assert algo != Automatic()
        end

        return new(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
    end
end

function Base.copy(mt::ModelType)
    return ModelType(mt.edge_obj, mt.agg_obj, mt.type, mt.cg, mt.algo, mt.unc, mt.uncparams)
end
