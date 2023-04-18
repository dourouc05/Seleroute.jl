# A parser for SNDlib instances, similar to Graph.jl's persistence layer or to
# GraphIO.jl. Terminology is closer to that of Graph.jl than to SNDlib: for
# instances, a topology is made of vertices and edges, not nodes and links.
# 
# All the metadata around the network graph is returned as a MetaGraph
# instance (MetaGraphs.jl) for the network topology, `g`, along with another
# MetaGraph instance for the demands, `k`. Here are the properties for `g`:
# - Graph has a name: :graph_name
# - Vertices have a unique ID: :vertex_id
# - Vertices have a location: :longitude and :latitude
# - Edges have a unique ID: :edge_id
# - Edges have a preinstalled capacity (potentially zero):
#   :preinstalled_capacity
# - Edges have a (useless) preinstalled cost: :preinstalled_cost
# - Edges have a routing cost (weight): :routing_cost
# - Edges have a setup cost if a nonzero capacity is installed: :setup_cost
# - Edges have a dictionary of installable capacities (key) and their
#   associated unit cost (value): :capacity_cost. An unbounded capacity is
#   represented as a `missing` capacity.
# - Demand edges have a unique ID: :demand_id
# - Demand edges have a routing unit: :routing_unit
# - Demand edges have a demand value (weight): :demand_value
# - Demand edges have a maximum path length: :max_path_length
#
# SNDlib graphs have a name; however, as only one graph is stored in each file,
# avoid checking the graph name as argument, rather store it as a graph property.
#
# Official description: http://sndlib.zib.de/html/docu/io-formats/
#
# Only version 1.0 is implemented. Features that are documented as part of
# version 2.0 will trigger errors.
#
# While the specification is lacking in terms of overall structure, this reader
# needs that the sections first indicate the vertices before anything else, so
# that it can perform some validation (add an edge only if the end vertices
# exist).
#
# Admissible paths are not supported.

struct SNDlibNativeFormat <: Graphs.AbstractGraphFormat end
struct SNDlibXMLFormat <: Graphs.AbstractGraphFormat end # Unimplemented for now.

@enum SNDlibNativeSection Header Meta Nodes Links Demands AdmissiblePaths
# The section "Header" is also used between two sections, when one is ended and
# another one has not yet started.

function parse_int_or_float(string::AbstractString)
    value = tryparse(Int, string)
    if value === nothing
        value = tryparse(Float64, string)
    end
    return value
end
function parse_int_or_float_or_unlimited(string::AbstractString)
    return if string == "UNLIMITED"
        missing
    else
        parse_int_or_float(string)
    end
end

function loadsnd(io::IO)
    REGEX_ID = raw"[a-zA-Z0-9][a-zA-Z0-9-_\.]*"
    REGEX_SPACE = "[ \t]+"
    REGEX_FLOAT = raw"-?\d+\.?\d*" # Either float or int.

    REGEX_NODE = Regex(
        "(?<id>$(REGEX_ID))$(REGEX_SPACE) # Vertex ID.
         \\( # Parenthesis group: position.
             ($(REGEX_SPACE))?
             (?<lat>$(REGEX_FLOAT))
             $(REGEX_SPACE)
             (?<lon>$(REGEX_FLOAT))
             ($(REGEX_SPACE))?
         \\)", "x")
    REGEX_LINK = Regex(
        "(?<id>$(REGEX_ID)) $(REGEX_SPACE)
         \\( # Parenthesis group: edge ends.
            ($(REGEX_SPACE))?
            (?<src>$(REGEX_ID))
            $(REGEX_SPACE)
            (?<dst>$(REGEX_ID))
            ($(REGEX_SPACE))?
        \\)
        $(REGEX_SPACE)
        (?<preinstalled_cap>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        (?<preinstalled_cost>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        (?<routing_cost>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        (?<setup_cost>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        \\(
            ($(REGEX_SPACE))?
            (?<capacity_cost_list>
                (
                    ($(REGEX_SPACE))?
                    ($(REGEX_FLOAT)|UNLIMITED)
                    $(REGEX_SPACE)
                    $(REGEX_FLOAT)
                    ($(REGEX_SPACE))?
                )*
            )
            ($(REGEX_SPACE))?
        \\)", "x")
    REGEX_DEMAND = Regex(
        "(?<id>$(REGEX_ID)) $(REGEX_SPACE)
         \\( # Parenthesis group: edge ends.
            $(REGEX_SPACE)?
            (?<src>$(REGEX_ID))
            $(REGEX_SPACE)
            (?<dst>$(REGEX_ID))
            $(REGEX_SPACE)?
        \\)
        $(REGEX_SPACE)
        (?<routing_unit>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        (?<demand_value>$(REGEX_FLOAT))
        $(REGEX_SPACE)
        (?<max_path_length>($(REGEX_FLOAT)|UNLIMITED))", "x")

    g = MetaDiGraph() # Topology.
    k = MetaDiGraph() # Demands.

    has_seen_header = false
    current_section = Header
    vertex_name_to_id = Dict{String, Int}() # Built in the Nodes section.

    for line in readlines(io)
        line = strip(line)
        line_stripped = line # Only useful for error messages.

        if length(line) == 0
            continue
        end

        # Handle comments. Ignore them, with few exceptions.
        if startswith(line, '#')
            line = strip(line[2:end])

            # Recognise the graph name in some comments (not standardised),
            # only in the header (not found anywhere else).
            if current_section == Header && startswith(line, "network")
                line = strip(line[8:end])
                set_prop!(g, :graph_name, line)
                continue
            end

            # Ignore other comments; no information to retrieve from them.
            continue
        end

        # Detect the section opening, if any.
        if current_section == Header && endswith(line, "(")
            if occursin("META", line)
                current_section = Meta
            elseif occursin("NODES", line)
                current_section = Nodes
            elseif occursin("LINKS", line)
                current_section = Links
            elseif occursin("DEMANDS", line)
                current_section = Demands
            elseif occursin("ADMISSIBLE_PATHS", line)
                current_section = AdmissiblePaths
            end
            continue
        end

        # Detect the section ending, if any.
        if current_section != Header && line == ")"
            current_section = Header
            continue
        end

        # Read the header line.
        if current_section == Header && startswith(line, '?') # Header.
            # Expected header (only networks supported):
            # ?SNDlib native format; type: network; version: 1.0
            EXPECTED_NO_WHITESPACE_HEADER = "SNDlibnativeformattypenetworkversion10"
            if replace(line, r"\W" => "") != EXPECTED_NO_WHITESPACE_HEADER
                error("Unexpected header line: \"", line_stripped, "\"")
            end

            has_seen_header = true
            continue
        end

        # Before reading anything in the instance, ensure that the header is
        # somewhere. Otherwise, it's possible that this instance contains some
        # 2.0 constructs, totally unsupported for now.
        if !has_seen_header
            error("Non-comment line before header: \"", line_stripped, "\"")
        end

        # All special cases are handled above: read the line as a part of the section.
        if current_section == Header
            error("Inconsistent state; while in Header section, encountered ",
                "the line: \"", line_stripped, "\"; expected a new section")
        elseif current_section == Meta
            property, value = split(line, '=', limit=2)
            property = strip(property)
            value = strip(value)
            set_prop!(g, Symbol(property), value)
        elseif current_section == Nodes
            # Node format:
            #     node_id ( x-coordinate y-coordinate )
            m = match(REGEX_NODE, line)
            if m === nothing
                error("Unexpected format when reading a vertex: \"",
                    line_stripped, "\"")
            end

            if ! add_vertex!(g)
                error("Could not add a new vertex to the topology graph")
            end
            if ! add_vertex!(k)
                error("Could not add a new vertex to the demand graph")
            end
            if nv(g) != nv(k)
                error("Assertion failed: different numbers of vertices in g and k; ",
                    nv(g), " ≠ ", nvg(k))
            end

            set_prop!(g, nv(g), :vertex_id, m["id"])
            set_prop!(g, nv(g), :longitude, parse_int_or_float(m["lon"]))
            set_prop!(g, nv(g), :latitude, parse_int_or_float(m["lat"]))
            set_prop!(k, nv(k), :vertex_id, m["id"])

            vertex_name_to_id[m["id"]] = nv(g)
        elseif current_section == Links
            # Link format:
            #     link_id ( source_id  target_id ) preinstalled_cap \
            #     preinstalled_cost routing_cost setup_cost \
            #     ( capacity_cost_list )
            m = match(REGEX_LINK, line)
            if m === nothing
                error("Unexpected format when reading a link: \"",
                    line_stripped, "\"")
            end

            src_id = vertex_name_to_id[m["src"]]
            dst_id = vertex_name_to_id[m["dst"]]

            capacity_costs =
                Dict{Union{Missing, Float64, Int}, Union{Float64, Int}}()
            capacity_costs_list =
                split(m["capacity_cost_list"], Regex(REGEX_SPACE))
            i = 1
            while i < length(capacity_costs_list)
                capacity = parse_int_or_float_or_unlimited(capacity_costs_list[i])
                cost = parse_int_or_float(capacity_costs_list[i + 1])

                capacity_costs[capacity] = cost

                i += 2
            end

            add_edge!(g, src_id, dst_id)
            set_prop!(g, Edge(src_id, dst_id), :edge_id, m["id"])
            set_prop!(g, Edge(src_id, dst_id), :preinstalled_capacity, parse_int_or_float(m["preinstalled_cap"]))
            set_prop!(g, Edge(src_id, dst_id), :preinstalled_cost, parse_int_or_float(m["preinstalled_cost"]))
            set_prop!(g, Edge(src_id, dst_id), :routing_cost, parse_int_or_float(m["routing_cost"]))
            set_prop!(g, Edge(src_id, dst_id), :setup_cost, parse_int_or_float(m["setup_cost"]))
            set_prop!(g, Edge(src_id, dst_id), :capacity_cost, capacity_costs)
        elseif current_section == Demands
            # Demand format:
            #     demand_id ( source_id  target_id ) routing_unit \ 
            #     demand_value max_path_length
            m = match(REGEX_DEMAND, line)
            if m === nothing
                error("Unexpected format when reading a demand edge: \"",
                    line_stripped, "\"")
            end

            src_id = vertex_name_to_id[m["src"]]
            dst_id = vertex_name_to_id[m["dst"]]

            add_edge!(k, src_id, dst_id)
            set_prop!(k, Edge(src_id, dst_id), :demand_id, m["id"])
            set_prop!(k, Edge(src_id, dst_id), :routing_unit, parse_int_or_float(m["routing_unit"]))
            set_prop!(k, Edge(src_id, dst_id), :demand_value, parse_int_or_float(m["demand_value"]))
            set_prop!(k, Edge(src_id, dst_id), :max_path_length, parse_int_or_float_or_unlimited(m["max_path_length"]))
        elseif current_section == AdmissiblePaths
            # Only case where admissible paths are expected to work: an empty
            # list. Rationale: how would you return them? A third value?
            error("Admissible paths are not supported")
        else
            error("Inconsistent state: ", current_section, " not expected")
        end
    end

    return g, k
end

loadgraph(io::IO, ::String, ::SNDlibNativeFormat) = loadsnd(io)
savegraph(io::IO, g::AbstractGraph, ::String, ::SNDlibNativeFormat) = savesnd(io, g)
