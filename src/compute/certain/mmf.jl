# TODO: column generation for all these functions.

function _master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction)
    rm = basic_routing_model_unitary(m, rd)
    capacity_constraints(rm, rd.traffic_matrix)

    @variable(m, τ)
    rm.tau = τ

    return rm
end

function master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MinMaxFair)
    rm = _master_mmf_problem(m, rd, edge_obj)

    @objective(m, Min, rm.tau)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) <= rm.tau)
    rm.constraints_mmf = mmf

    return rm
end

function master_mmf_problem(m::Model, rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::MaxMinFair)
    rm = _master_mmf_problem(m, rd, edge_obj)

    @objective(m, Max, rm.tau)
    @constraint(m, mmf[e in edges(rd)], objective_edge_expression(rm, edge_obj, e) >= rm.tau)
    rm.constraints_mmf = mmf

    return rm
end

function compute_routing(rd::RoutingData, edge_obj::EdgeWiseObjectiveFunction, agg_obj::Union{MinMaxFair, MaxMinFair}, type::FormulationType, cg::Val{false}, ::Automatic, ::NoUncertaintyHandling, ::NoUncertainty)
    # Implementation of the water-filling algorithm for MMF routings (aka MFMF).
    # https://ica1www.epfl.ch/PS_files/LEB3132.pdf
    # https://www.utc.fr/~dnace/recherche/Publication/Inoc03Nace.pdf

    start = time_ns()

    # Create the master problem.
    m = _create_model(rd)
    rm = master_mmf_problem(m, rd, edge_obj, edge_agg)
    time_create_master_model_ms = (time_ns() - start) / 1_000_000.

    # Memorise the evolution of the algorithm.
    n_iter = 0
    routings = Routing[]
    objectives = Float64[]
    times_ms = Float64[]
    end_status = MOI.OPTIMAL

    # Iteratively solve it by removing τ constraints and fixing the values of
    # the corresponding objectives.
    edges_to_do = Set(edges(rd))
    fixed_objectives = Dict{Edge{Int}, Float64}()

    while length(edges_to_do) > 0
        start = time_ns()

        # Optimise for this iteration.
        optimize!(m)
        status = termination_status(m)
        _export_lp_if_allowed(rd, m, "lp_$(n_iter)")

        # Handle abnormal cases.
        if status != MOI.OPTIMAL
            # OPTIMAL status is shown with the objective value.
            rd.logmessage("[$(n_iter)] Status: $(status).")

            # Debug infeasibility if need be.
            _debug_infeasibility_mmf(rd, status, n_iter, edge_obj, agg_obj, type, cg, algo, unc, uncparams)

            # Report other problematic codes.
            if status in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED] && n_iter > 0
                end_status = MOI.ALMOST_OPTIMAL
                break
            end
        end

        # Report the values for this iteration.
        master_τ = objective_value(m)
        rd.logmessage("[$(n_iter)] Status: $(status). Value: $(master_τ)")
        push!(routings, Routing(rd, value.(rm.routing)))
        push!(objectives, master_τ)

        # Find edges to fix.
        new_edges = _mmf_find_edges_to_fix(rm, master_τ)
        rd.logmessage("  Number of variables to fix: $(length(new_edges)).")

        # Debugging when no edge met the conditions.
        if length(new_edges) == 0
            if n_iter == 0
                # No variable can be fixed right now, there must be a problem!
                @warn "No variable can be fixed at the first iteration, " *
                      "the MMF process stalls."
            else
                # Every link seems to be saturated, end now.
                rd.logmessage("[$(n_iter)] All links are saturated.")
            end
        end

        # Add the new constraints for the next iteration.
        for (e, value) in new_edges
            # Maintain the data structures.
            fixed_objectives[e] = value
            pop!(edges_to_do, e)

            # Modify the master to indicate that the value is now known.
            _mmf_fix_edge(rm, e, value, agg_obj)
        end

        rd.logmessage("  Edges not yet saturated: $(length(edges_to_do)). Total edges saturated: $(length(fixed_objectives))")

        # Log a few things at the end of the iteration. Stop if no new edge was
        # found.
        n_iter += 1
        push!(times_ms, (time_ns() - start) / 1_000_000.)

        if length(new_edges) == 0
            # The user is already warned.
            break
        end
    end

    # Not necessary to optimise one last time: the remaining edges were fixed
    # to the current value in the master (i.e. the last iteration just added
    # redundant constraints).

    return RoutingSolution(rd, result=end_status,
                           time_precompute_ms=rd.time_precompute_ms,
                           time_create_master_model_ms=time_create_master_model_ms,
                           time_solve_ms=times_ms,
                           objectives=objectives,
                           matrices=rd.traffic_matrix,
                           routings=routings,
                           master_model=rm)
end

function _mmf_find_edges_to_fix(rm::RoutingModel, master_τ::Float64)
    # Check which constraints are satisfied at equality (i.e. nonzero dual
    # value, as the constraint is either ≥ or ≤).
    new_edges = Dict{Edge{Int}, Float64}()

    if has_duals(rm.model)
        for e in edges_to_do
            if - agg_obj.ε <= dual(rm.constraints_mmf[e]) <= agg_obj.ε
                new_edges[e] = master_τ
            end
        end
    end

    if length(new_edges) == 0
        # If no edge goes through the dual variable test, check the actual
        # values (this may be due to the way the objective function is
        # modelled, like Fortz-Thorup, or if the solver doesn't give duals).
        for e in edges_to_do
            if - agg_obj.ε <= value(rm.constraints_mmf[e]) <= agg_obj.ε
                new_edges[e] = master_τ
            end
        end
    end

    return new_edges
end

function _mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, rhs::Float64)
    rm.data.logmessage("  Fixing $e to $value ≈ $rhs (relaxed value)")
    set_normalized_coefficient(rm.constraints_mmf[e], τ, 0)
    set_normalized_rhs(rm.constraints_mmf[e], rhs)
end

_mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, agg_obj::MinMaxFair) =
    _mmf_fix_edge(rm, r, value, value * (1 + agg_obj.ε))

_mmf_fix_edge(rm::RoutingModel, e::Edge{Int}, value::Float64, agg_obj::MaxMinFair) =
    _mmf_fix_edge(rm, r, value, value * (1 - agg_obj.ε))

# Debug infeasiblity. Implement the main logic here. Throw an error in case of
# infeasibility.
function _debug_infeasibility_mmf(rd::RoutingData, status::MOI.TerminationStatusCode, n_iter::Int,
                                  edge_obj::EdgeWiseObjectiveFunction, Union{MinMaxFair, MaxMinFair},
                                  type::FormulationType, cg::Val, algo::Automatic,
                                  unc::NoUncertaintyHandling, uncparams::NoUncertainty)
    if status in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED] && n_iter == 0
        rd.logmessage("[$(n_iter)] This iteration of MMF yielded an infeasible optimisation program. ")
        d = _debug_infeasibility_mmf(rd, edge_obj, agg_obj, type, cg, algo, unc, uncparams)
        if d !== nothing
            rd.logmessage("[$(n_iter)] The source of infeasibility could be automatically detected: ")
            rd.logmessage("[$(n_iter)] $d")
        else
            rd.logmessage("[$(n_iter)] The source of infeasibility could not be automatically detected.")
        end

        # Exit with an error.
        _export_lp_if_failed(rd, status, m, "error_$(n_iter)", "Current problem could not be solved!")
    end
end

_debug_infeasibility_mmf(::RoutingData, ::EdgeWiseObjectiveFunction,
                         ::MinMaxFair, ::FormulationType, ::Val, ::Automatic,
                         ::NoUncertaintyHandling, ::NoUncertainty) = nothing

function _debug_infeasibility_mmf(rd::RoutingData, ::KleinrockLoad,
                                  ::MinMaxFair, type::FormulationType,
                                  ::Val{false}, ::Automatic,
                                  ::NoUncertaintyHandling, ::NoUncertainty)
    # Main source of infeasibility: impossible to have links with a load less
    # than 100%.
    # Technique: minimise the maximum load (i.e. not Kleinrock), check if it
    # is 100%.
    r = compute_routing(rd, Load(), MinimumMaximum(), type, Val(false),
                        Automatic(), NoUncertaintyHandling(), NoUncertainty())
    if compute_max_load(r) >= 0.99999
        return "This network cannot withstand this traffic matrix: " *
               "its maximum load is 100% or more (i.e. the most loaded link is " *
               "loaded at 100%, when minimising this maximum load). " *
               "Kleinrock function associates an infinite penalty to links " *
               "with such a high load."
    end
    return nothing
end
