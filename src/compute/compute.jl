"""
Based on an opaque `RoutingData` object `rd` and a solver (for instance, `CPLEX.Optimizer`), computes a routing.
It returns one object, a `RoutingSolution`, containing all intermediary results.
"""
function compute_routing(rd::RoutingData)
    # Only perform dispatch on the type of model.
    mt = rd.model_type
    return compute_routing(rd, Val(mt.edge_obj), Val(mt.agg_obj), Val(mt.type), Val(mt.cg), Val(mt.algo), Val(mt.unc), Val(mt.uncparams))
end

function compute_routing(rd::RoutingData, edge_obj::Val, agg_obj::Val, type::Val, cg::Val, algo::Val, unc::Val, uncparams::Val)
    # When dispatch fails, use this fallback.
    msg = "Model type not yet implemented: ModelType($edge_obj, $agg_obj, $type, $cg, $algo, $unc, $uncparams).\n"
    msg *= "These models are available:\n"
    msg *= string(methods(compute_routing))
    error(msg)
end
