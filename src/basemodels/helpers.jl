function _create_model(rd::RoutingData)
    m = Model(rd.solver)
    set_silent(m) # TODO: add a parameter for this
    return m
end
