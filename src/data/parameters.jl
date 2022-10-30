"""
Opaque data structure holding the whole set of parameters used to compute a routing.

# Internals

These are implementation details that could evolve at any time. Many parameters here are already documented in the main
public constructor for `RoutingData`.

The first four fields are the most dependable of all of them (i.e. they are likely to stay unchanged for a long period of time):

* `g`: topology.
* `k`: demands.
* `solver`.
* `name`.

Several fields configure the solving process.

* Some parameters are always used:
  * `model_type`.
  * `model_simplifications`. TODO: reintroduce them.
  * `timeout`.
* Some parameters are sometimes used:
  * `sub_model_type`.
* Some parameters are only useful for oblivious routing:
  * `model_all_traffic_matrices`: only for cutting-plane implementation.
  * `model_exact_opt_d`: only for cutting-plane implementation.
  * `model_robust_reformulation_traffic_matrices`: only for dual-reformulation implementation.
* Some parameters are only useful for problems without uncertainty.
  * `traffic_matrix`.

Several fields configure the output process.

* `logmessage`
* `plot_final_results`: TODO: reintroduce.
* `plot_each_iteration`: TODO: reintroduce.
* `export_lps`
* `export_lps_on_error`
* `output_folder`

From these graphs, several data structures are derived. Some of them are lazily computed when required (hence the
`mutable` aspect of this structure).

* For path-based formulations, paths are stored and uniquely identified:
  * `paths_edges`: all the paths, represented as list of edges (Edges). Path indices are always consistent
    with `paths`. Used as follows: `paths_edges[path_id]` is a list of edges.
  * `demand_to_path_ids`: maps a demand (arc from source to destination) to its path IDs.
  * `path_id_to_demand`: maps a path ID to the corresponding demand.
* In order to draw the graph, node positions are computed at most once:
  * `locs_x`: for each node, its x coordinate when drawing the graph (lazily created).
  * `locs_y`: for each node, its y coordinate when drawing the graph (lazily created).
  * `locs_f`: function that computes the two previous fields when requested. Takes a graph as input and outputs
    two vectors, `locs_x` and `locs_y`.

The time to create the `RoutingData` object is memorised in the `time_precompute_ms` field.
"""
mutable struct RoutingData
    g::MetaDiGraph
    k::AbstractGraph
    solver
    name::String
    time_precompute_ms::Float64

    model_type::ModelType
    sub_model_type::ModelType
    model_simplifications::Bool
    model_all_traffic_matrices::Bool
    model_exact_opt_d::Bool
    model_robust_reformulation_traffic_matrices::Bool
    timeout::Period
    enable_variable_constraint_names::Bool

    traffic_matrix::Dict{Edge{Int}, Float64}

    paths_edges::Vector{Vector{Edge}}
    demand_to_path_ids::Dict{Edge{Int}, Vector{Int}}
    path_id_to_demand::Dict{Int, Edge}

    locs_x::Union{Vector{Float64}, Nothing}
    locs_y::Union{Vector{Float64}, Nothing}
    locs_f

    logmessage
    plot_final_results
    plot_each_iteration
    export_lps
    export_lps_on_error
    output_folder
end

"""
Creates an opaque `RoutingData` object based on the topology of the network `g` and the demands to route `k`.
Calling this function may take a while, as it precomputes many things used later on, based on the type of model to use.
Not all parameters are used in all cases.

Parameters that are always used (only the first three are required and positional):

* `g` is a directed graph with annotation (hence the type `MetaDiGraph`). Each edge is supposed to have a property
  `capacity` giving its capacity (units are not imposed by the package, but must be consistent). Each node must
  have a `name` property, which will be used for generating the output.
* `k` is another directed graph, without necessarily annotations (hence the type `AbstractSimpleGraph`).
* `solver` is the optimization solver to use for this problem (like `CPLEX.Optimizer`).
* `model_type` is the type of optimisation model used to compute the oblivious routing. Its type is `ModelType`.
* `name` is an optional name for the model.

The parameters almost always apply, and may have tremendous impact on solving performance:

* `model_simplifications` enables simplifications in the model. They do not imply any kind of approximation.
  Computing what parts of the model can be simplified may take more time than what the simplifications may improve
  the running times of the optimalisation part (especially for small networks and those with very high connexity).
* `remove_unreachable_nodes` determines whether the nodes in the topology `g` that are isolated, i.e. those that
  have no neighbours, should be removed
* `remove_unsatisfiable_demands` determines whether the demands that are not routable (i.e. there is no path in `g`
  from their source to their destination, the topology `g` provides no connectivity between these two nodes) should
  be removed.
* `timeout`: the maximum time for the computations. `Second(0)` indicates that there is no limit. If the time limit
  is reached, the resulting solution will have the `MOI.TIME_LIMIT` status code with the current solution: it will
  have no optimality or feasibility guarantee!
* `enable_variable_constraint_names`: whether variables and constraints should have names attached to them. Enabling
  this option (the default) eases the debugging of the models; disabling it improves performance, especially for
  large models.

Some models may use these parameters:

* `sub_model_type` is only used by algorithms using subproblems, like oblivious routing with cutting planes.
  All the following parameters are reused for the subproblems: `model_simplifications`, `npaths`.
* `npaths` is the number of shortest paths that should be computed beforehand for path-based formulations, solved with
  column generation or not. This number of paths is considered *for each demand*: there will be
  probably more than `npaths` that get generated, but never more than `npaths` times the number of demands).
  By default, 20 paths at most are generated for each demand.
  For column-generation-based formulations, this provides a starting set of columns: this set may be increased in size
  during the solving process. Otherwise, this is the total number of paths that will be used for solving the problem.

These parameters are only used for oblivious routing:

* `model_all_traffic_matrices` indicates whether all traffic matrices that violate the oblivious routing constraints
  should be added or not, at each iteration of the master problem. (The original algorithm only adds one matrix,
  but adding more at a time may improve running times by reducing the total number of iterations to perform.)
* `model_exact_opt_d` indicates whether, in the added constraints, the optimal congestion is computed for the generated
  traffic matrix. In theory, this value is always 1.0.
* `model_robust_reformulation_traffic_matrices` enables computing traffic matrices for the robust reformulations.
  Using this options increases the number of constraints in the problem to solve in order to get the right dual values.

These parameters are only used for problems without uncertainty.

* `traffic_matrix` is the only traffic matrix that is considered.

These parameters only control the output of the solving process:

* `locs_f` function that computes the position of the nodes when plotting the graph in 2D. It takes a graph as input and
  outputs two vectors, first the x positions, then the y positions.
* `logmessage` is a function that will be called whenever the package produces any textual output. By default,
  text is redirected to the shell (`println`). This function should take a single string argument, the message to print.
* `verbose` indicates whether this function (not the complete package) should be explicit about what it is doing
  (for instance, if it removes demands). This option does not include regular progress logging for long operations.
* `plot_final_results`: whether plots should be produced when the solving process is done.
* `plot_each_iteration`: for iterative algorithms, whether plots should be produced at each iteration.
* `export_lps`: export the LP files (if possible) for each and every (sub)problem that is solved. If the master problem
  is iteratively built, each iteration will be exported. As the LP format can only represent MILP problems, not all
  combinations of problem types and solving algorithms can benefit from this parameter.
* `export_lps_on_error`: export the LP files (if possible) of the (sub)problem being solved when an error occurs.
  As the LP format can only represent MILP problems, not all combinations of problem types and solving algorithms
  can benefit from this parameter.
* `output_folder`: the folder where all file output is supposed to be taking place. This folder must exist beforehand
  or be an empty string to disable all output.
"""
function RoutingData(g::AbstractMetaGraph, # Not MetaDiGraph, to show a more interesting error message if the graph is not directed.
                     k::AbstractGraph,
                     solver,
                     model_type::ModelType;
                     name::String="",
                     sub_model_type::Union{ModelType, Nothing}=nothing,
                     model_simplifications::Bool=false,
                     model_all_traffic_matrices::Bool=false,
                     model_exact_opt_d::Bool=false,
                     model_robust_reformulation_traffic_matrices::Bool=false,
                     timeout::Period=Second(0),
                     enable_variable_constraint_names::Bool=true,
                     traffic_matrix::Dict{Edge{Int}, Float64}=Dict{Edge{Int}, Float64}(),
                     npaths::Int=20,
                     remove_unreachable_nodes::Bool=true,
                     remove_unsatisfiable_demands::Bool=true,
                     locs_f=spring_layout,
                     logmessage=println,
                     verbose::Bool=true,
                     plot_final_results::Bool=false,
                     plot_each_iteration::Bool=false,
                     export_lps::Bool=false,
                     export_lps_on_error::Bool=false,
                     output_folder::String="")
    start = time_ns()

    if sub_model_type === nothing
        sub_model_type = copy(model_type)
    end

    # Check whether the right properties have been defined in the graphs.
    for v in vertices(g)
        if ! has_prop(g, v, :name)
            error("Node $(v) has no name!")
        end
    end
    for e in edges(g)
        if ! has_prop(g, e, :capacity)
            error("Edge $(e) has no capacity!")
        end
    end

    if ! is_directed(g)
        error("The demand graph is not directed!")
    end
    if ! is_directed(k)
        error("The demand graph is not directed!")
    end

    # Check whether the other parameters have easily-checkable properties, before more cumbersome computations.
    if output_folder != "" && ! isdir(output_folder)
        error("The output path $(output_folder) does not exist!")
    end

    # Copy the graphs locally, so that the caller does not see when they are modified internally
    # (for instance, due to remove_unsatisfiable_demands).
    g = copy(g)
    k = copy(k)

    # If need be, remove some vertices because they are isolated.
    if remove_unreachable_nodes
        nremoved = remove_unreachable_nodes!(g)
        if verbose && nremoved > 0
            logmessage("Removed $(nremoved) vertices because they were not reachable.")
        end
    end

    # If need be, remove some demands due to lack of connectivity.
    if remove_unsatisfiable_demands
        nremoved = remove_unsatisfiable_demands!(g, k)
        if verbose && nremoved > 0
            logmessage("Removed $(nremoved) demands because they were not routable.")
        end
    end

    # If this is a path-based formulation, it may require some path generation.
    if (model_type.type == PathFormulation() || sub_model_type.type == PathFormulation()) && npaths > 0
        # Precompute some shortest paths and directly assign them numbers.
        # A path is a sequence of integers, each being the index of a node.
        paths = Vector{Int}[]
        demand_to_path_ids = Dict{Edge{Int}, Vector{Int}}()
        path_id_to_demand = Dict{Int, Edge}()

        for d in edges(k)
            demand_paths = yen_k_shortest_paths(g, src(d), dst(d), weights(g), npaths).paths
            if ! (model_type.cg || sub_model_type.cg) && length(demand_paths) == npaths
                @warn("The limit of paths to generate ($(npaths)) has been reached; the result will not be guaranteed to be optimum. " *
                      "Switch to a column-generation formulation for exact results.")
            end

            demand_to_path_ids[d] = Int[]
            for path in demand_paths
                push!(paths, path)
                p_id = length(paths)
                push!(demand_to_path_ids[d], p_id)
                path_id_to_demand[p_id] = d
            end
        end

        # Paths are returned as a list of nodes, transform it into a list of edges.
        paths_edges = [map(t -> Edge(t[1], t[2]), zip(p[1:end-1], p[2:end])) for p in paths]
    else
        # Not a path-based formulation: create empty collections of the right type.
        paths_edges = Vector{Edge}[]
        demand_to_path_ids = Dict{Edge{Int}, Vector{Int}}()
        path_id_to_demand = Dict{Int, Edge}()
    end

    # For the special case of reformulation and column generation for oblivious
    # routing, retrieving traffic matrices disables simplifications; otherwise,
    # if the matrices are not required, enable the simplifications.
    if model_type.agg_obj == MinimumMaximum() &&
            model_type.type == PathFormulation() &&
            model_type.algo == DualReformulation() &&
            model_type.unc == ObliviousUncertainty() &&
            model_type.uncparams == UncertainDemand() &&
            ! model_simplifications
        model_simplifications = ! model_robust_reformulation_traffic_matrices
    end
    # TODO: keep this code here or move it to a warning/error in the
    # corresponding solver?

    stop = time_ns()

    return RoutingData(g, k, solver, name, (stop - start) / 1_000_000.,
                       # Solving parameters.
                       model_type, sub_model_type, model_simplifications,
                       model_all_traffic_matrices, model_exact_opt_d,
                       model_robust_reformulation_traffic_matrices,
                       timeout, enable_variable_constraint_names, traffic_matrix,
                       # Precomputed things.
                       paths_edges, demand_to_path_ids, path_id_to_demand,
                       # Output parameters (graph plotting and logging).
                       nothing, nothing, locs_f, logmessage, plot_final_results,
                       plot_each_iteration, export_lps, export_lps_on_error, output_folder)
end

function remove_unreachable_nodes!(g::AbstractGraph)
    # First, determine which nodes are unreachable; then, delete them.
    # If deletion is made in the loop over all vertices, as soon as a deletion
    # is performed, the vertex indices may change: vertices(g) is no more
    # consistent with inneighbors(g, v) and outneighbors(g, v).
    to_remove = Int[]
    for v in vertices(g)
        if length(inneighbors(g, v)) == 0 && length(outneighbors(g, v)) == 0
            push!(to_remove, v)
        end
    end
    for v in to_remove
        rem_vertex!(g, v)
    end
    return length(to_remove)
end

function remove_unsatisfiable_demands!(g::AbstractGraph, k::AbstractGraph)
    to_remove = Edge{Int}[]
    for e in edges(k)
        paths = dijkstra_shortest_paths(g, src(e), weights(g))
        if length(paths.parents) < dst(e) || paths.parents[dst(e)] == 0
            push!(to_remove, e)
        end
    end
    for e in to_remove
        rem_edge!(k, e)
    end
    return length(to_remove)
end

function add_path!(rd::RoutingData, demand::Edge, path::Vector{Edge})
    # Update RoutingData with the new path.
    # All demands already have a path, by assumption. Otherwise, the
    # master problem should not be feasible. Hence, computing the new
    # path ID is easy: just take the length of the vector.
    push!(rd.paths_edges, path)
    path_id = length(rd.paths_edges)

    rd.path_id_to_demand[path_id] = demand
    push!(rd.demand_to_path_ids[demand], path_id)

    return path_id
end

graph(rd::RoutingData) = rd.g
edges(rd::RoutingData) = edges(rd.g)
vertices(rd::RoutingData) = vertices(rd.g)
demands(rd::RoutingData) = edges(rd.k)

n_nodes(rd::RoutingData) = nv(rd.g)
n_edges(rd::RoutingData) = ne(rd.g)
n_demands(rd::RoutingData) = ne(rd.k)
n_paths(rd::RoutingData) = length(rd.paths_edges)

capacity(rd::RoutingData, e::Edge{Int}) = get_prop(rd.g, e, :capacity)

function find_path_ids_with_edge(rd::RoutingData, e::Edge)
    if n_paths(rd) > 0
        return filter(1:n_paths(rd)) do p
            e in rd.paths_edges[p]
        end
    else
        error("Precomputed paths are not available. Are you trying to use this function in a flow-based formulation?")
    end
end
