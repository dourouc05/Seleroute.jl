"""
The edge-wise objective function to optimise. These values are possible:

* `Load`: the load of each edge is considered.
* `KleinrockLoad`: Kleinrock function is applied on the load.
* `FortzThorupLoad`: Fortz-Thorup linearisation of Kleinrock function is applied on the load.

Kleinrock function is defined as:

``K(x) = \\frac{x}{1-x}``
"""
@enum EdgeWiseObjectiveFunction begin
    Load
    KleinrockLoad
    FortzThorupLoad
end

"""
The aggregation-function objective to optimise, giving one objective function aggregating
all the components for each edge (indicated by `EdgeWiseObjectiveFunction`). These values are possible:

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
@enum AggregationObjectiveFunction begin
    MinimumTotal
    MinimumMaximum
    MinMaxFair
end

"""
The type of formulation that is used to solve the routing problem. There are two main families of formulations:

* `FlowFormulation`: each edge is explicitly present in the model
* `PathFormulation`: the model uses path variables. The path-based formulations can be solved in two ways:
  * precomputing paths: only the set of precomputed paths is considered; if not enough paths are precomputed,
    the solution is only approximate, and no approximation factor is guaranteed
  * column generation: new paths are added on the fly, when they might improve the solution
"""
@enum FormulationType begin
    FlowFormulation
    PathFormulation
end

"""
For robust and oblivious uncertainties, there are multiple algorithm choices:

* `Automatic`: let the package decide which algorith to use
* `CuttingPlane`: use an iterative cutting-plane algorithm (solves many small problems)
* `DualReformulation`: use a dual-reformulation of the uncertainty set (solves one large problem)

For other types of uncertainty, use the value `Automatic`.
"""
@enum AlgorithmChoice begin
    Automatic
    CuttingPlane
    DualReformulation
    # TODO: Different algorithms for MMF?
end

"""
The way uncertainty is implemented. These values are possible:

* `NoUncertaintyHandling`: the uncertainty is not taken into account, only nominal values are used
* `StochasticUncertainty`: the uncertainty is implemented through stochastic optimisation,
  i.e. with several (potentially weighted) values of the unknown parameters
* `RobustUncertainty`: the uncertainty is implemented through robust optimisation,
  i.e. the unknown parameters belong to a given (bounded) uncertainty set
* `ObliviousUncertainty`: the uncertainty is implemented through oblivious optimisation,
  i.e. the unknown parameters belong to an unbouded uncertainty set
"""
@enum UncertaintyHandling begin
    NoUncertaintyHandling
    StochasticUncertainty
    RobustUncertainty
    ObliviousUncertainty
end

"""
The parameters of the problem that can see uncertainty. These values are possible:

* `NoUncertainty`: there is no uncertainty in the problem
* `UncertainDemand`: the demand to route are not fully known, but the topology has no uncertainty
* `UncertainCapacity`: the links have unknown capacities, but the demands to route are fully known
"""
@enum UncertainParameters begin
    NoUncertainty
    UncertainDemand
    UncertainCapacity
end

"""
Parameterising the way a routing-optimisation problem must be solved. Four parameters must be chosen (independently
of each other):

* an objective function, of type `ObjectiveFunction`
* a formulation type, of type `FormulationType` (for path-based formulations, column generation can be separately enabled)
* a way to heandle the uncertainty, of type `UncertaintyHandling`
* a precise algorithm to solve the instance
* a definition of which parameters can be afflicted by uncertainty, of type `UncertainParameters`

The exact algorithm used to solve the instance is based on those four parameters.
"""
struct ModelType
    edge_obj::EdgeWiseObjectiveFunction
    agg_obj::AggregationObjectiveFunction
    type::FormulationType
    cg::Bool
    algo::AlgorithmChoice
    unc::UncertaintyHandling
    uncparams::UncertainParameters

    function ModelType(edge_obj::EdgeWiseObjectiveFunction, agg_obj::AggregationObjectiveFunction, type::FormulationType, cg::Bool,
                       algo::AlgorithmChoice, unc::UncertaintyHandling, uncparams::UncertainParameters)
        is_obj_load = edge_obj in [Load, KleinrockLoad, FortzThorupLoad]
        if is_obj_load && uncparams == UncertainCapacity
            error("Load-related edge-wise objective functions like $(edge_obj) are not supposed to be used with capacity uncertainty.")
        end

        if type == FlowFormulation && cg
            error("Column generation is not defined for flow formulations")
        end

        if (unc != ObliviousUncertainty && unc != RobustUncertainty) && algo != Automatic
            # TODO: MMF?
            error("Different algorithms are only available for robust- and oblivious-uncertainty models.")
        end
        if (unc == ObliviousUncertainty || unc == RobustUncertainty) && algo == Automatic
            algo = DualReformulation # Usually the most efficient one. Could implement heuristics based on numerical experiments.
        end

        if (unc == NoUncertaintyHandling && uncparams != NoUncertainty) || (unc == NoUncertaintyHandling && uncparams != NoUncertainty)
            text = "When there is no uncertainty, the parameters UncertaintyHandling"
            if unc != NoUncertaintyHandling && uncparams == NoUncertainty
                text *= " ($unc != $NoUncertaintyHandling)"
            end
            text *= " and UncertainParameters"
            if unc == NoUncertaintyHandling && uncparams != NoUncertainty
                text *= " ($uncparams != $NoUncertainty)"
            end
            text *= " must match."
            error(text)
        end

        if uncparams == UncertainCapacity
            error("No model implemented uncertainty on link capacity")
        end

        # Check that the parameter detection worked.
        if unc == ObliviousUncertainty || unc == RobustUncertainty
            @assert algo != Automatic
        end

        return new(edge_obj, agg_obj, type, cg, algo, unc, uncparams)
    end
end

function Base.copy(mt::ModelType)
    return ModelType(mt.edge_obj, mt.agg_obj, mt.type, mt.cg, mt.algo, mt.unc, mt.uncparams)
end
