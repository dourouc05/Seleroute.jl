function export_traffic_matrix(tm::ConsideredTrafficMatrix, basename="$(@__DIR__)")
    export_traffic_matrix(tm.matrix, "$(basename)_$(tm.subproblem_edge.src)_to_$(tm.subproblem_edge.dst)")
end

function export_traffic_matrix(tm::Dict{Edge{Int}, Float64}, filename::String)
    open(filename, "w") do f
        write(f, "Source, Destination, Traffic\n")
        for (edge, value) in tm
            write(f, "$(edge.src), $(edge.dst), $(value)\n")
        end
    end
end

"""
Summarises as text the differences between two different routings.
"""
function summary(rd::RoutingData, base_flows, oblivious_flows; filename="$(@__DIR__)/summary.txt", epsilon=1.e-2)
    # Helper functions.
    stringNode(n) = "$(get_prop(rd.g, n, :name))"
    stringEdge(rd::Tuple) = "($(stringNode(rd[1])), $(stringNode(rd[2])))"
    stringEdge(rd::Edge) = "($(stringNode(src(rd))), $(stringNode(dst(rd))))"
    stringPath(p) = join(map(stringEdge, p), ", ")

    # Get rid of JuMP's data structures.
    base_routing_matrix = zeros(n_demands(rd), n_paths(rd))
    for k in eachindex(base_flows)
        base_routing_matrix[k[1], k[2]] = base_flows[k...]
    end
    oblivious_routing_matrix = zeros(n_demands(rd), n_paths(rd))
    for k in eachindex(oblivious_flows)
        oblivious_routing_matrix[k[1], k[2]] = oblivious_flows[k...]
    end

    # Write the summary.
    list_edges = collect(edges(rd.g))
    open(filename, "w") do f
        write(f, "Total load\n")
        write(f, "==========\n\n")

        base_path_loads = sum(base_routing_matrix[d, :] for d in 1:n_demands(rd))
        oblivious_path_loads = sum(oblivious_routing_matrix[d, :] for d in 1:n_demands(rd))

        base_edge_throughputs = [any(in(e, rd.paths_edges[p]) for p in findall(base_path_loads .> 0)) ? sum(base_path_loads[p] * in(e, rd.paths_edges[p]) for p in findall(base_path_loads .> 0)) : 0.0 for e in 1:n_edges(rd)]
        oblivious_edge_throughputs = [any(in(e, rd.paths_edges[p]) for p in findall(oblivious_path_loads .> 0)) ? sum(oblivious_path_loads[p] * in(e, rd.paths_edges[p]) for p in findall(oblivious_path_loads .> 0)) : 0.0 for e in 1:n_edges(rd)]

        base_edge_loads = [base_edge_throughputs[i] / get_prop(rd.g, list_edges[i], :capacity) for i in 1:n_edges(rd)]
        oblivious_edge_loads = [oblivious_edge_throughputs[i] / get_prop(rd.g, list_edges[i], :capacity) for i in 1:n_edges(rd)]

        write(f, "Load    | Base    | Oblivious\n")
        write(f, "Maximum |  $(@sprintf("% 2.2f", 100 * maximum(base_edge_loads)))  | $(@sprintf("% 2.2f", 100 * maximum(oblivious_edge_loads)))\n")
        write(f, "Average |  $(@sprintf("% 2.2f", 100 * mean(base_edge_loads)))  | $(@sprintf("% 2.2f", 100 * mean(oblivious_edge_loads)))\n")
        write(f, "Minimum |  $(@sprintf("% 2.2f", 100 * minimum(base_edge_loads)))  | $(@sprintf("% 2.2f", 100 * minimum(oblivious_edge_loads)))\n")
        write(f, "\n")

        for i in 1:n_edges(rd)
            s = src(list_edges[i])
            t = dst(list_edges[i])

            write(f, "  - Edge #$(i): $(stringEdge((s, t)))\n") # TODO: Write a specialisation of stringEdge that directly works with edges?
            write(f, "    Base throughput: $(round(base_edge_throughputs[i], digits=2)) Mbps\n")
            write(f, "    Base load: $(round(100 * base_edge_loads[i], digits=2))%\n")
            write(f, "    Oblivious throughput: $(round(oblivious_edge_throughputs[i], digits=2)) Mbps\n")
            write(f, "    Oblivious load: $(round(100 * oblivious_edge_loads[i], digits=2))%\n")
        end

        write(f, "\n\n\n")

        for d in 1:n_demands(rd)
            k_d = rd.demand_id_to_demand[rd.path_id_to_demand_id[d]]

            write(f, "Demand #$(d): from $(src(k_d)) to $(dst(k_d))\n")
            write(f, "======\n\n")

            # When a route is used with a weight higher than epsilon, consider it is used (boolean).
            base_routes = vec(copy(base_routing_matrix[d, :]))
            base_routes[base_routes .>= epsilon] .= 1
            base_routes[base_routes .< epsilon] .= 0
            oblivious_routes = vec(copy(oblivious_routing_matrix[d, :]))
            oblivious_routes[oblivious_routes .>= epsilon] .= 1
            oblivious_routes[oblivious_routes .< epsilon] .= 0

            base_n_routes = sum(base_routes)
            oblivious_n_routes = sum(oblivious_routes)

            # Statistics on the number of possible paths for this demand (be they actually used or not).
            write(f, "Number of possible paths for this demand: $(length(rd.demand_id_to_path_ids[d]))\n")
            write(f, "Minimum path length (number of hops): $(minimum(map(p -> length(rd.paths[p]), rd.demand_id_to_path_ids[d])))\n")
            write(f, "Average path length (number of hops): $(round(mean(map(p -> length(rd.paths[p]), rd.demand_id_to_path_ids[d])), digits=2))\n")
            write(f, "Maximum path length (number of hops): $(maximum(map(p -> length(rd.paths[p]), rd.demand_id_to_path_ids[d])))\n")
            write(f, "\n")

            # Compare the number of routes.
            if base_n_routes != oblivious_n_routes
                write(f, "Base and oblivious solutions do not have the same number of routes: \n")
                write(f, "  - base: $(convert(Int, base_n_routes)) route$((base_n_routes != 1.0) ? "s" : "")\n")
                write(f, "  - oblivious: $(convert(Int, oblivious_n_routes)) route$((oblivious_n_routes != 1.0) ? "s" : "")\n")
            else
                write(f, "Base and oblivious solutions have the same number of routes. \n")
            end

            # Compare the actual routes.
            if any(base_routes .!= oblivious_routes)
                write(f, "Base and oblivious solutions do not have the same routes: \n")

                for r in 1:length(base_routes)
                    if base_routes[r] == 0 && oblivious_routes[r] == 1
                        write(f, "  - difference for route #$(r): used only by oblivious\n")
                        write(f, "    $(stringPath(rd.paths_edges[r]))\n")
                    elseif base_routes[r] == 1 && oblivious_routes[r] == 0
                        write(f, "  - difference for route #$(r): used only by base\n")
                        write(f, "    $(stringPath(rd.paths_edges[r]))\n")
                    end
                end
            else
                write(f, "Base and oblivious solutions have the same routes. \n")
            end

            # Compare the route weights.
            if norm(vec(base_routing_matrix[d, :]) - vec(oblivious_routing_matrix[d, :])) >= epsilon
                write(f, "Base and oblivious solutions do not have the same route weights: \n")

                for r in 1:nPaths
                    if abs(base_routing_matrix[d, r] - oblivious_routing_matrix[d, r]) >= epsilon
                        write(f, "  - significant difference for route #$(r): base $(round(base_routing_matrix[d, r], digits=2)), oblivious $(round(oblivious_routing_matrix[d, r], digits=2))\n")
                        write(f, "    $(stringPath(rd.paths_edges[r]))\n")
                    end
                end
            else
                write(f, "Base and oblivious solutions have the same route weights. \n")
            end

            write(f, "\n")

            # Loads of the links for each used path.
            write(f, "Loads for the base routing: \n")
            for p in rd.demand_id_to_path_ids[d]
                if base_routing_matrix[d, p] <= epsilon
                    continue
                end

                write(f, "  - Path #$(p): \n")

                for (i, e) in enumerate(rd.paths_edges[p])
                    write(f, "      - $(stringEdge(e)): $(base_edge_loads[i])\n")
                end
            end

            write(f, "Loads for the oblivious routing: \n")
            for p in rd.demand_id_to_path_ids[d]
                if oblivious_routing_matrix[d, p] <= epsilon
                    continue
                end

                write(f, "  - Path #$(p): \n")

                for (i, e) in enumerate(rd.paths_edges[p])
                    write(f, "      - $(stringEdge(e)): $(oblivious_edge_loads[i])\n")
                end
            end

            write(f, "\n\n\n")
        end
    end
end

function export_routing(routing::Routing, filename="$(@__DIR__)/oblivious_routing.txt"; epsilon=1.e-2)
    rd = routing.data
    nodeName(n) = get_prop(rd.g, n, :name)

    routing_matrix = routing_to_matrix(routing)
    open(filename, "w") do f
        write(f, "Demand number, Path proportion, Vertices (names), Edges (name origin - name destination)\n")
        for d in 1:n_demands(rd)
            for p in findall(x -> x > epsilon, routing_matrix[d, :])
                write(f, "$(d), $(routing_matrix[d, p]), ")

                # Path as a list of vertices.
                for edge in routing.paths[p]
                    write(f, "$(nodeName(src(edge))) ")
                end
                write(f, "$(nodeName(dst(routing.paths[p][end]))), ")

                # Path as a list of edges.
                for edge in routing.paths[p]
                    write(f, "($(nodeName(src(edge))) - $(nodeName(dst(edge))))")
                end

                write(f, "\n")
            end
        end
    end
end
