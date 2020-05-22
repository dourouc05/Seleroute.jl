"""
Based on an opaque `RoutingData` object `rd` and a solver (for instance, `CPLEX.Optimizer`), computes a routing.
It returns one object, a `RoutingSolution`, containing all intermediary results.
"""
function compute_routing(rd::RoutingData)
    # Only perform dispatch on the type of model.
    mt = rd.model_type
    return compute_routing(rd, mt.edge_obj, mt.agg_obj, mt.type, Val(mt.cg), mt.algo, mt.unc, mt.uncparams)
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::AggregationObjectiveFunction,
                         type::FormulationType, cg::Val, algo::AlgorithmChoice, unc::UncertaintyHandling, uncparams::UncertainParameters)
    # When dispatch fails, use this fallback.
    msg = "Model type not yet implemented: ModelType($edge_obj, $agg_obj, $type, $cg, $algo, $unc, $uncparams).\n"
    msg *= "These models are available:\n"
    msg *= string(methods(compute_routing))
    error(msg)
end
