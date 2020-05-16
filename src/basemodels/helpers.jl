function _find_path_ids_with_edge(rd::RoutingData, e::Edge)
    if n_paths(rd) > 0
        return filter(1:n_paths(rd)) do p
            in(e, rd.paths_edges[p])
        end
    else
        error("Precomputed paths are not available. Are you trying to use this function in a flow-based formulation?")
    end
end
