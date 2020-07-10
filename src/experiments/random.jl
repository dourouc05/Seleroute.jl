function random_similar(demands::Vector{Edge}, avg::Float64, stdev::Float64, rng)
    # rng is assumed to have a normal distribution (i.e. both positive and
    # negative values can be generated from it).
    values = avg .+ stdev .* randn(rng, length(demands))
    values[values .< 0.0] .= 1.0 # Remove negative traffic value, they make no sense.
    return Dict(demands[i] => values[i] for i in length(demands))
end

random_similar(demands::Dict{Edge{Int}, Float64}, avg::Float64, stdev::Float64, rng) =
    random_similar(collect(keys(demands)), avg, stdev, rng)

random_similar(rd::RoutingData, avg::Float64, stdev::Float64, rng) =
    random_similar(collect(demands(rd)), avg, stdev, rng)

function random_experiment(rd::RoutingData, solver, oblivious_routing::Routing,
                           avg::Float64, stdev::Float64;
                           rng=MersenneTwister())
    traffic = random_similar(rd, avg, stdev, rng)

    # Compute the optimum load for this traffic matrix, with a routing that
    # is perfectly adapted to it.
    m = _create_model(rd)
    capacitated_demand_routing_model(m, rd, traffic)
    optimize!(m)
    optimum_load = objective_value(m)

    # Maybe we generated an infeasible matrix, so the results would make no sense. Generate a new matrix.
    if optimum_load > 1.0
        return random_experiment(rd, solver, oblivious_routing, avg, stdev, rng=rng)
    end

    oblivious_load = compute_max_load(oblivious_routing, traffic)
    return oblivious_load, optimum_load
end

function random_experiments(rd::RoutingData, solver, oblivious_routing,
                           avg::Float64, stdev::Float64, n::Int=1; kwargs...)
    oblivious_loads = Float64[]
    optimum_loads = Float64[]
    for _ in 1:n
        experiment = random_experiment(rd, solver, oblivious_routing, avg, stdev; kwargs...)
        push!(oblivious_loads, experiment[1])
        push!(optimum_loads, experiment[2])
    end

    return oblivious_loads, optimum_loads, oblivious_loads ./ optimum_loads
end
