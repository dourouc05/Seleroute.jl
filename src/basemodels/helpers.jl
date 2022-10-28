function _create_model(rd::RoutingData)
    m = Model(rd.solver)
    set_string_names_on_creation(m, rd.enable_variable_constraint_names)
    set_silent(m) # TODO: add a parameter for this
    return m
end
