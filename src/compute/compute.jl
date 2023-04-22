"""
Based on a `RoutingData` object `rd` and a solver (for instance,
`CPLEX.Optimizer`), computes a routing. It returns one object, a
`RoutingSolution`, containing all intermediary results.

There is no way to get rid of the intermediate results during the solving
process. The rationale is twofold: compared to the LP solvers, these results
shouldn't be too large; Seleroute is first and foremost a research code, the
major use case is to analyse the intermediate results.
"""
function compute_routing(rd::RoutingData)
    mt = rd.model_type # Only perform dispatch on the type of model.
    return compute_routing(rd, mt.edge_obj, mt.agg_obj, mt.type, Val(mt.cg),
                           mt.algo, mt.unc, mt.uncparams)
end

function compute_routing(::RoutingData, edge_obj::EdgeWiseObjectiveFunction,
                         agg_obj::AggregationObjectiveFunction,
                         type::FormulationType, cg::Val, algo::AlgorithmChoice,
                         unc::UncertaintyHandling,
                         uncparams::UncertainParameters)
    # When dispatch fails, use this fallback to give the user better feedback
    # than the standard Julia error.
    msg = "Model type not yet implemented: ModelType($edge_obj, $agg_obj, "
    msg *= "$type, $cg, $algo, $unc, $uncparams).\n"
    msg *= "These models are available:\n"
    msg *= string(methods(compute_routing))
    error(msg)
end
