function _find_path_ids_with_edge(rd::RoutingData, e::Edge)
    if n_paths(rd) > 0
        return filter(1:n_paths(rd)) do p
            in(e, rd.paths_edges[p])
        end
    else
        error("Precomputed paths are not available. Are you trying to use this function in a flow-based formulation?")
    end
end

function _create_model(rd::RoutingData)
    m = Model(rd.solver)
    set_silent(m) # TODO: add a parameter for this
    return m
end
