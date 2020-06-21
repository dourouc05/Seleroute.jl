using Seleroute

oneGbps = 1024
halfGbps = 500 # Many topologies will reach a 100% load with 512. 

topologies = Vector{Topology}()

# First tested topology.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [2, 3, .9 * oneGbps] [2, 6, oneGbps] [3, 4, oneGbps] [3, 6, oneGbps] [5, 6, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("basic", nodesMatrix, edgesMatrix, trafficMatrix))

### "Olver", inspired by http://math.mit.edu/~olver/dynamic.pdf, Figure 1.
## Basic topology, with uniform capacities.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 7, oneGbps] [2, 5, oneGbps] [2, 7, oneGbps] [3, 4, oneGbps] [3, 5, oneGbps] [3, 7, oneGbps] [4, 6, oneGbps] [4, 7, oneGbps] [5, 6, oneGbps] [5, 7, oneGbps] [6, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant, with nonuniform capacities.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, 5 * oneGbps] [1, 3, halfGbps] [1, 7, 2 * oneGbps] [2, 5, 0.21 * oneGbps] [3, 4, 4.96 * oneGbps] [3, 5, 2.47 * oneGbps] [3, 7, 5 * oneGbps] [4, 6, oneGbps] [4, 7, oneGbps] [5, 6, oneGbps] [5, 7, oneGbps] [6, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-various", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant, with other capacities (lower).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps / 5] [1, 3, halfGbps] [2, 5, oneGbps / 0.21] [3, 4, oneGbps / 4.96] [3, 5, oneGbps / 2.47] [4, 6, oneGbps / 0.147] [5, 6, oneGbps / 5] [1, 7, 2 * oneGbps] [2, 7, 0.147 * oneGbps] [3, 7, 5 * oneGbps] [5, 7, oneGbps] [6, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-various-inv-partial", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant, with other capacities (higher).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, 5 * oneGbps] [1, 3, 2 * oneGbps] [2, 5, 0.21 * oneGbps] [3, 4, 4.96 * oneGbps] [3, 5, 2.47 * oneGbps] [4, 6, 0.147 * oneGbps] [5, 6, 5 * oneGbps] [1, 7, oneGbps / 2] [2, 7, oneGbps / 0.147] [3, 7, oneGbps / 5] [5, 7, oneGbps / 2] [6, 7, oneGbps / 2]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-various-inv", nodesMatrix, edgesMatrix, trafficMatrix))

## Olver, but without the node G (it has a very high degree).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 5, oneGbps] [3, 4, oneGbps] [3, 5, oneGbps] [4, 6, oneGbps] [5, 6, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-without_G", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant, with nonuniform capacities.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, 5 * oneGbps] [1, 3, halfGbps] [2, 5, 0.21 * oneGbps] [3, 4, 4.96 * oneGbps] [3, 5, 2.47 * oneGbps] [4, 6, 0.147 * oneGbps] [5, 6, 5 * oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-without_G-various", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant, with other capacities (lower).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps / 5] [1, 3, halfGbps] [2, 5, oneGbps / 0.21] [3, 4, oneGbps / 4.96] [3, 5, oneGbps / 2.47] [4, 6, oneGbps / 0.147] [5, 6, oneGbps / 5]]'
trafficMatrix = [[1, 3, halfGbps] [1, 6, halfGbps] [4, 1, halfGbps] [4, 5, halfGbps] [5, 2, halfGbps] [6, 4, halfGbps]]'

push!(topologies, Topology("olver-without_G-various-inv", nodesMatrix, edgesMatrix, trafficMatrix))

### Topology Zoo
# Abilene (http://www.topology-zoo.org/files/Abilene.gml).
# Traffic: https://totem.info.ucl.ac.be/dataset.html
# (IntraTM-2005-02-23-21-00.xml); removed self-traffic.
nodesMatrix = permutedims([[1, "NY"] [2, "CH"] [3, "DC"] [4, "SE"] [5, "SU"] [6, "LA"] [7, "DE"] [8, "KA"] [9, "HO"] [10, "AT"] [11, "IN"]], [2, 1])
edgesMatrix = [[1, 2, 10 * oneGbps] [1, 3, 10 * oneGbps] [2, 11, 10 * oneGbps] [3, 10, 10 * oneGbps] [4, 5, 10 * oneGbps] [4, 7, 10 * oneGbps] [5, 6, 10 * oneGbps] [5, 7, 10 * oneGbps] [6, 9, 10 * oneGbps] [7, 8, 10 * oneGbps] [8, 9, 10 * oneGbps] [8, 11, 10 * oneGbps] [9, 10, 10 * oneGbps] [10, 11, 10 * oneGbps]]'
trafficMatrix = [[8, 4, 6240.1600] [8, 1, 26863.6533] [8, 5, 28226.0267] [8, 3, 47.3956] [8, 1, 2817.4489] [8, 7, 6438.9689] [8, 2, 7656.7911] [8, 6, 4.7111] [8, 9, 1867315.3956] [4, 8, 21866.3111] [4, 1, 4646580.7022] [4, 5, 41628.4000] [4, 3, 59591.7956] [4, 7, 1285829.3600] [4, 2, 1196886.1244] [4, 6, 23.1911] [4, 11, 1926.3911] [4, 9, 63727.9378] [1, 8, 598001.9289] [1, 4, 342504.4533] [1, 5, 14935.0756] [1, 3, 355844.4622] [1, 7, 16094485.4311] [1, 2, 2543737.4400] [1, 6, 3683.2800] [1, 11, 7058.1422] [1, 9, 171177.7600] [5, 8, 35060.1244] [5, 4, 3344.5778] [5, 1, 29604.1867] [5, 3, 86.2400] [5, 10, 5342.3289] [5, 7, 123.6089] [5, 2, 27998.6044] [5, 6, 606.5778] [5, 11, 26.6667] [5, 9, 778276.2578] [3, 8, 27451.4844] [3, 4, 1958030.1956] [3, 1, 843953.6711] [3, 5, 106844.8000] [3, 10, 65148.4178] [3, 7, 369093.9733] [3, 2, 207656.8711] [3, 6, 1749.4133] [3, 11, 24723.9644] [3, 9, 69562.1511] [10, 8, 549654.6933] [10, 1, 63275.7778] [10, 5, 767704.7467] [10, 3, 82480.2756] [10, 7, 95816.9867] [10, 2, 115608.1778] [10, 6, 137330.3022] [10, 11, 81214.3556] [10, 9, 1256926.2222] [7, 8, 41063.8311] [7, 4, 1366652.5244] [7, 1, 2448645.0933] [7, 5, 83908.4711] [7, 3, 310578.6222] [7, 10, 29714.8444] [7, 2, 941842.7822] [7, 6, 8392.0000] [7, 11, 46217.5644] [7, 9, 57574.9067] [2, 8, 43894.6400] [2, 4, 335900.4000] [2, 1, 158223.1644] [2, 5, 16902.1867] [2, 3, 78114.3022] [2, 10, 177692.9600] [2, 7, 74661.2089] [2, 6, 421889.7778] [2, 11, 13222.6756] [2, 9, 18675.6622] [6, 4, 1389.6178] [6, 1, 62036.3289] [6, 5, 193.7067] [6, 3, 2093.6889] [6, 10, 1693.0844] [6, 7, 8646.7733] [6, 2, 364485.7333] [6, 9, 80837.0489] [11, 8, 6725.6444] [11, 4, 4773.3067] [11, 1, 32356.2933] [11, 5, 24303.0578] [11, 3, 53.0489] [11, 10, 9530.7822] [11, 7, 4520.6133] [11, 2, 400782.9511] [11, 6, 0.3556] [11, 9, 248961.2711] [9, 8, 652315.8311] [9, 4, 34186.6578] [9, 1, 21203.9911] [9, 5, 13540.1333] [9, 3, 19.8044] [9, 10, 720.1956] [9, 7, 837.6711] [9, 2, 18402.5333] [9, 11, 9.4578]]'
trafficMatrix[:, 3] /= 1024 * 3 # Mbps
trafficMatrix = round.(Int, trafficMatrix, RoundDown)

push!(topologies, Topology("abilene", nodesMatrix, edgesMatrix, trafficMatrix))

# ATT (http://www.topology-zoo.org/files/AttMpls.gml).
# No traffic data available (take the same as Abilene).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"] [15, "O"] [16, "P"] [17, "Q"] [18, "R"] [19, "S"] [20, "T"] [21, "U"] [22, "V"] [23, "W"] [24, "X"] [25, "Y"] [26, "Z"]], [2, 1])
edgesMatrix = [[1, 2, 10 * oneGbps] [1, 3, 10 * oneGbps] [1, 17, 10 * oneGbps] [2, 3, 10 * oneGbps] [3, 4, 10 * oneGbps] [3, 5, 10 * oneGbps] [4, 5, 10 * oneGbps] [3, 17, 10 * oneGbps] [5, 17, 10 * oneGbps] [3, 6, 10 * oneGbps] [3, 26, 10 * oneGbps] [3, 14, 10 * oneGbps] [3, 7, 10 * oneGbps] [7, 8, 10 * oneGbps] [8, 9, 10 * oneGbps] [7, 9, 10 * oneGbps] [9, 10, 10 * oneGbps] [6, 26, 10 * oneGbps] [5, 6, 10 * oneGbps] [6, 17, 10 * oneGbps] [6, 14, 10 * oneGbps] [10, 11, 10 * oneGbps] [10, 14, 10 * oneGbps] [11, 14, 10 * oneGbps] [11, 12, 10 * oneGbps] [11, 13, 10 * oneGbps] [12, 13, 10 * oneGbps] [12, 14, 10 * oneGbps] [13, 19, 10 * oneGbps] [14, 19, 10 * oneGbps] [14, 18, 10 * oneGbps] [14, 15, 10 * oneGbps] [14, 16, 10 * oneGbps] [15, 16, 10 * oneGbps] [15, 26, 10 * oneGbps] [16, 26, 10 * oneGbps] [17, 26, 10 * oneGbps] [16, 17, 10 * oneGbps] [16, 18, 10 * oneGbps] [16, 19, 10 * oneGbps] [18, 19, 10 * oneGbps] [19, 20, 10 * oneGbps] [19, 21, 10 * oneGbps] [17, 22, 10 * oneGbps] [17, 23, 10 * oneGbps] [18, 22, 10 * oneGbps] [21, 23, 10 * oneGbps] [22, 23, 10 * oneGbps] [22, 24, 10 * oneGbps] [24, 25, 10 * oneGbps]]'
trafficMatrix = [[8, 4, 6240.1600] [8, 1, 26863.6533] [8, 5, 28226.0267] [8, 3, 47.3956] [8, 1, 2817.4489] [8, 7, 6438.9689] [8, 2, 7656.7911] [8, 6, 4.7111] [8, 9, 1867315.3956] [4, 8, 21866.3111] [4, 1, 4646580.7022] [4, 5, 41628.4000] [4, 3, 59591.7956] [4, 7, 1285829.3600] [4, 2, 1196886.1244] [4, 6, 23.1911] [4, 11, 1926.3911] [4, 9, 63727.9378] [1, 8, 598001.9289] [1, 4, 342504.4533] [1, 5, 14935.0756] [1, 3, 355844.4622] [1, 7, 16094485.4311] [1, 2, 2543737.4400] [1, 6, 3683.2800] [1, 11, 7058.1422] [1, 9, 171177.7600] [5, 8, 35060.1244] [5, 4, 3344.5778] [5, 1, 29604.1867] [5, 3, 86.2400] [5, 10, 5342.3289] [5, 7, 123.6089] [5, 2, 27998.6044] [5, 6, 606.5778] [5, 11, 26.6667] [5, 9, 778276.2578] [3, 8, 27451.4844] [3, 4, 1958030.1956] [3, 1, 843953.6711] [3, 5, 106844.8000] [3, 10, 65148.4178] [3, 7, 369093.9733] [3, 2, 207656.8711] [3, 6, 1749.4133] [3, 11, 24723.9644] [3, 9, 69562.1511] [10, 8, 549654.6933] [10, 1, 63275.7778] [10, 5, 767704.7467] [10, 3, 82480.2756] [10, 7, 95816.9867] [10, 2, 115608.1778] [10, 6, 137330.3022] [10, 11, 81214.3556] [10, 9, 1256926.2222] [7, 8, 41063.8311] [7, 4, 1366652.5244] [7, 1, 2448645.0933] [7, 5, 83908.4711] [7, 3, 310578.6222] [7, 10, 29714.8444] [7, 2, 941842.7822] [7, 6, 8392.0000] [7, 11, 46217.5644] [7, 9, 57574.9067] [2, 8, 43894.6400] [2, 4, 335900.4000] [2, 1, 158223.1644] [2, 5, 16902.1867] [2, 3, 78114.3022] [2, 10, 177692.9600] [2, 7, 74661.2089] [2, 6, 421889.7778] [2, 11, 13222.6756] [2, 9, 18675.6622] [6, 4, 1389.6178] [6, 1, 62036.3289] [6, 5, 193.7067] [6, 3, 2093.6889] [6, 10, 1693.0844] [6, 7, 8646.7733] [6, 2, 364485.7333] [6, 9, 80837.0489] [11, 8, 6725.6444] [11, 4, 4773.3067] [11, 1, 32356.2933] [11, 5, 24303.0578] [11, 3, 53.0489] [11, 10, 9530.7822] [11, 7, 4520.6133] [11, 2, 400782.9511] [11, 6, 0.3556] [11, 9, 248961.2711] [9, 8, 652315.8311] [9, 4, 34186.6578] [9, 1, 21203.9911] [9, 5, 13540.1333] [9, 3, 19.8044] [9, 10, 720.1956] [9, 7, 837.6711] [9, 2, 18402.5333] [9, 11, 9.4578]]'
trafficMatrix[:, 3] /= 1024 * 3 # Mbps
trafficMatrix = round.(Int, trafficMatrix, RoundDown)

push!(topologies, Topology("att", nodesMatrix, edgesMatrix, trafficMatrix))

### Purely synthetic graphs.
## Path graph.
# Oblivious competitive ratio should be 1.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [2, 3, oneGbps] [3, 4, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 4, halfGbps]]'

push!(topologies, Topology("path", nodesMatrix, edgesMatrix, trafficMatrix))

## Binary trees.
# Pure tree, seven nodes. Full binary, depth of 3.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps] [4, 5, halfGbps]]'

push!(topologies, Topology("full_binary_tree", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Connect leaves only within the subtree. Same demands as before
# (one demand within a subtree is likely to be impacted, as it can be served
# directly).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [4, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps] [6, 7, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps]]'

push!(topologies, Topology("full_binary_tree-connected_leaves_subtree", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Connect only leaves, be they in the same subtree or not.
# Same demands as before, more changes expected.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [4, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps] [6, 7, oneGbps] [5, 6, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps]]'

push!(topologies, Topology("full_binary_tree-connected_leaves", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Connect only the two inner nodes 2 and 3 (depth of 2).
# Same demands as before, many demands can go through the new link.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps]]'

push!(topologies, Topology("full_binary_tree-connected_inner", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Connect only the two inner nodes 2 and 3 (depth of 2) with a
# high-capacity link (impossible to saturate). Same demands as before,
# many demands can go through the new link.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 3, 10 * oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps]]'

push!(topologies, Topology("full_binary_tree-connected_inner-high-capacity", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Connect only the two inner nodes 2 and 3 (depth of 2) with a
# high-capacity link (impossible to saturate). Same demands as before,
# many demands can go through the new link.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"]], [2, 1])
edgesMatrix = [[1, 2, 5 * oneGbps] [1, 3, 7 * oneGbps] [2, 3, 20 * oneGbps] [2, 4, 3 * oneGbps] [2, 5, 2 * oneGbps] [3, 6, 3 * oneGbps] [3, 7, 8 * oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, oneGbps] [6, 4, 2 * oneGbps] [7, 5, oneGbps]]'

push!(topologies, Topology("full_binary_tree-connected_inner-high-capacity-various", nodesMatrix, edgesMatrix, trafficMatrix))

## Two "binary trees", connected together.
# Two trees, full binary, depth of 3. Connected only at their root (i.e.
# full binary tree of depth 4, in this case). Symmetric demands for the two
# graphs, plus a few crossing the trees. First tree rooted at 1, second at 8.
# Inspired by https://epubs.siam.org/doi/abs/10.1137/S0895479896312262.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps] [8, 9, oneGbps] [8, 10, oneGbps] [9, 11, oneGbps] [9, 12, oneGbps] [10, 13, oneGbps] [10, 14, oneGbps] [1, 8, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps] [4, 5, halfGbps] [8, 14, halfGbps] [9, 13, halfGbps] [13, 11, halfGbps] [14, 12, halfGbps] [11, 12, halfGbps] [2, 9, halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are fully
# connected. Symmetric demands for the two graphs, plus a few crossing the trees
# (unchanged from 11). First tree rooted at 1, second at 8.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps] [8, 9, oneGbps] [8, 10, oneGbps] [9, 11, oneGbps] [9, 12, oneGbps] [10, 13, oneGbps] [10, 14, oneGbps] [1, 8, oneGbps] [2, 9, oneGbps] [3, 10, oneGbps] [4, 11, oneGbps] [5, 12, oneGbps] [6, 13, oneGbps] [7, 14, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps] [4, 5, halfGbps] [8, 14, halfGbps] [9, 13, halfGbps] [13, 11, halfGbps] [14, 12, halfGbps] [11, 12, halfGbps] [2, 9, halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-fully_connected", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are fully
# connected. Symmetric demands for the two graphs, plus a few crossing the trees
# (unchanged from 11). First tree rooted at 1, second at 8.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, 14.24 * oneGbps] [1, 3, .01 * oneGbps] [2, 4, pi * oneGbps] [2, 5, 10 * oneGbps] [3, 6, oneGbps / pi] [3, 7, .001 * oneGbps] [8, 9, 10 * oneGbps] [8, 10, .01 * oneGbps] [9, 11, oneGbps] [9, 12, 10 * oneGbps] [10, 13, oneGbps] [10, 14, oneGbps] [1, 8, 10 * oneGbps] [2, 9, oneGbps] [3, 10, oneGbps] [4, 11, 10 * oneGbps] [5, 12, oneGbps] [6, 13, oneGbps] [7, 14, 10 * oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, .1 * halfGbps] [6, 4, halfGbps] [7, 5, 2.4 * halfGbps] [4, 5, halfGbps] [8, 14, 5 * halfGbps] [9, 13, halfGbps] [13, 11, halfGbps] [14, 12, 1.85 * halfGbps] [11, 12, halfGbps] [2, 9, halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-fully_connected-various", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are
# connected (a few edges do not exist, the ones that correspond to demands:
# 2-9 for demand 2-9, 3-10 for demand 10-2). Symmetric demands for the
# two graphs, plus a few crossing the trees (unchanged from 11). First tree
# rooted at 1, second at 8.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [3, 7, oneGbps] [8, 9, oneGbps] [8, 10, oneGbps] [9, 11, oneGbps] [9, 12, oneGbps] [10, 13, oneGbps] [10, 14, oneGbps] [1, 8, oneGbps] [4, 11, oneGbps] [5, 12, oneGbps] [6, 13, oneGbps] [7, 14, oneGbps]]'
trafficMatrix = [[1, 7, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [7, 5, halfGbps] [4, 5, halfGbps] [8, 14, halfGbps] [9, 13, halfGbps] [13, 11, halfGbps] [14, 12, halfGbps] [11, 12, halfGbps] [2, 9, halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-connected", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are
# connected (a few edges do not exist, the ones that correspond to demands:
# 2-9 for demand 2-9, 3-10 for demand 10-2). Symmetric demands for the two
# graphs, plus a few crossing the trees (unchanged from 11). First tree rooted
# at 1, second at 8.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, 10 * oneGbps] [1, 3, 5 * oneGbps] [2, 4, oneGbps] [2, 5, 10 * oneGbps] [3, 6, .05 * oneGbps] [3, 7, oneGbps] [8, 9, 10 * oneGbps] [8, 10, 5 * oneGbps] [9, 11, oneGbps] [9, 12, 10 * oneGbps] [10, 13, 5 * oneGbps] [10, 14, oneGbps] [1, 8, 10 * oneGbps] [4, 11, 5 * oneGbps] [5, 12, oneGbps] [6, 13, 10 * oneGbps] [7, 14, 5 * oneGbps]]'
trafficMatrix = [[1, 7, 5 * halfGbps] [2, 6, halfGbps] [6, 4, 5.2 * halfGbps] [7, 5, halfGbps] [4, 5, .5 * halfGbps] [8, 14, halfGbps] [9, 13, 5 * halfGbps] [13, 11, halfGbps] [14, 12, 5 * halfGbps] [11, 12, halfGbps] [2, 9, 5 * halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-connected-various", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are
# connected (a few edges do not exist, the ones that correspond to demands:
# 2-9 for demand 2-9, 3-10 for demand 10-2). Symmetric demands for the two
# graphs, plus a few crossing the trees (unchanged from 11). First tree rooted
# at 1, second at 8. Much lower capacities than usual.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, 10 * oneGbps] [1, 3, 5 * oneGbps] [2, 4, oneGbps] [2, 5, 10 * oneGbps] [3, 6, .005 * oneGbps] [3, 7, oneGbps] [8, 9, 10 * oneGbps] [8, 10, 5 * oneGbps] [9, 11, oneGbps] [9, 12, 10 * oneGbps] [10, 13, 5 * oneGbps] [10, 14, oneGbps] [1, 8, 10 * oneGbps] [4, 11, 5 * oneGbps] [5, 12, oneGbps] [6, 13, 10 * oneGbps] [7, 14, 5 * oneGbps]]'
trafficMatrix = [[1, 7, 5 * halfGbps] [2, 6, halfGbps] [6, 4, 5.2 * halfGbps] [7, 5, halfGbps] [4, 5, .5 * halfGbps] [8, 14, halfGbps] [9, 13, 5 * halfGbps] [13, 11, halfGbps] [14, 12, 5 * halfGbps] [11, 12, halfGbps] [2, 9, 5 * halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-connected-various_very_low", nodesMatrix, edgesMatrix, trafficMatrix))

# Variant. Two trees, full binary, depth of 3. The corresponding nodes are
# connected (a few edges do not exist, the ones that correspond to demands:
# 2-9 for demand 2-9, 3-10 for demand 10-2). Symmetric demands for the two
# graphs, plus a few crossing the trees (unchanged from 11). First tree rooted
# at 1, second at 8. Much lower capacities than usual.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"] [13, "M"] [14, "N"]], [2, 1])
edgesMatrix = [[1, 2, 10 * oneGbps] [1, 3, 5 * oneGbps] [2, 4, oneGbps] [2, 5, 10 * oneGbps] [3, 6, .0005 * oneGbps] [3, 7, oneGbps] [8, 9, 10 * oneGbps] [8, 10, 5 * oneGbps] [9, 11, oneGbps] [9, 12, 10 * oneGbps] [10, 13, 5 * oneGbps] [10, 14, oneGbps] [1, 8, 10 * oneGbps] [4, 11, 5 * oneGbps] [5, 12, oneGbps] [6, 13, 10 * oneGbps] [7, 14, 5 * oneGbps]]'
trafficMatrix = [[1, 7, 5 * halfGbps] [2, 6, halfGbps] [6, 4, 5.2 * halfGbps] [7, 5, halfGbps] [4, 5, .5 * halfGbps] [8, 14, halfGbps] [9, 13, 5 * halfGbps] [13, 11, halfGbps] [14, 12, 5 * halfGbps] [11, 12, halfGbps] [2, 9, 5 * halfGbps] [10, 2, halfGbps]]'

push!(topologies, Topology("double_tree-connected-various_very_very_low", nodesMatrix, edgesMatrix, trafficMatrix))

## Clique of 6 nodes.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps] [2, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [2, 6, oneGbps] [3, 4, oneGbps] [3, 5, oneGbps] [3, 6, oneGbps] [4, 5, oneGbps] [4, 6, oneGbps] [5, 6, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps]]'

push!(topologies, Topology("clique", nodesMatrix, edgesMatrix, trafficMatrix))

## Two clique of 6 nodes.
# Two cliques of 6 nodes, connected once. Symmetrical demands, plus a few that
# cross the connecting link.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps] [2, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [2, 6, oneGbps] [3, 4, oneGbps] [3, 5, oneGbps] [3, 6, oneGbps] [4, 5, oneGbps] [4, 6, oneGbps] [5, 6, oneGbps] [7, 8, oneGbps] [7, 9, oneGbps] [7, 10, oneGbps] [7, 11, oneGbps] [7, 12, oneGbps] [8, 9, oneGbps] [8, 10, oneGbps] [8, 11, oneGbps] [8, 12, oneGbps] [9, 10, oneGbps] [9, 11, oneGbps] [9, 12, oneGbps] [10, 11, oneGbps] [10, 12, oneGbps] [11, 12, oneGbps] [1, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps] [7, 9, halfGbps] [8, 12, halfGbps] [12, 10, halfGbps] [7, 11, halfGbps] [10, 11, halfGbps] [1, 7, halfGbps] [2, 8, halfGbps] [9, 3, halfGbps] [10, 4, halfGbps]]'

push!(topologies, Topology("double_clique", nodesMatrix, edgesMatrix, trafficMatrix))

# Two cliques of 6 nodes, connected 4 times (4 end-points in the first clique,
# 1 end-point in the second).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps] [2, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [2, 6, oneGbps] [3, 4, oneGbps] [3, 5, oneGbps] [3, 6, oneGbps] [4, 5, oneGbps] [4, 6, oneGbps] [5, 6, oneGbps] [7, 8, oneGbps] [7, 9, oneGbps] [7, 10, oneGbps] [7, 11, oneGbps] [7, 12, oneGbps] [8, 9, oneGbps] [8, 10, oneGbps] [8, 11, oneGbps] [8, 12, oneGbps] [9, 10, oneGbps] [9, 11, oneGbps] [9, 12, oneGbps] [10, 11, oneGbps] [10, 12, oneGbps] [11, 12, oneGbps] [1, 7, oneGbps] [3, 7, oneGbps] [4, 7, oneGbps] [5, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps] [7, 9, halfGbps] [8, 12, halfGbps] [12, 10, halfGbps] [7, 11, halfGbps] [10, 11, halfGbps] [1, 7, halfGbps] [2, 8, halfGbps] [9, 3, halfGbps] [10, 4, halfGbps]]'

push!(topologies, Topology("double_clique-connected", nodesMatrix, edgesMatrix, trafficMatrix))

## Star of 6 nodes.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps]]'

push!(topologies, Topology("star", nodesMatrix, edgesMatrix, trafficMatrix))

## Two stars of 6 nodes.
# Two stars of 6 nodes, connected once. Symmetrical demands, plus a few that
# cross the connecting link.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps] [7, 8, oneGbps] [7, 9, oneGbps] [7, 10, oneGbps] [7, 11, oneGbps] [7, 12, oneGbps] [1, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps] [7, 9, halfGbps] [8, 12, halfGbps] [12, 10, halfGbps] [7, 11, halfGbps] [10, 11, halfGbps] [1, 7, halfGbps] [2, 8, halfGbps] [9, 3, halfGbps] [10, 4, halfGbps]]'

push!(topologies, Topology("double_star", nodesMatrix, edgesMatrix, trafficMatrix))

# Two stars of 6 nodes, connected 4 times (4 end-points in the first star,
# 1 end-point in the star; never the centre of the star).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [1, 4, oneGbps] [1, 5, oneGbps] [1, 6, oneGbps] [7, 8, oneGbps] [7, 9, oneGbps] [7, 10, oneGbps] [7, 11, oneGbps] [7, 12, oneGbps] [1, 7, oneGbps] [3, 7, oneGbps] [4, 7, oneGbps] [5, 7, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps] [7, 9, halfGbps] [8, 12, halfGbps] [12, 10, halfGbps] [7, 11, halfGbps] [10, 11, halfGbps] [1, 7, halfGbps] [2, 8, halfGbps] [9, 3, halfGbps] [10, 4, halfGbps]]'

push!(topologies, Topology("double_star-connected", nodesMatrix, edgesMatrix, trafficMatrix))

## Grid of 12 nodes.
# 3 rows and 4 columns. Edges along rows or columns (no diagonals).
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "G"] [8, "H"] [9, "I"] [10, "J"] [11, "K"] [12, "L"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [2, 3, oneGbps] [3, 4, oneGbps] [1, 5, oneGbps] [5, 6, oneGbps] [6, 7, oneGbps] [7, 8, oneGbps] [2, 6, oneGbps] [3, 7, oneGbps] [4, 8, oneGbps] [5, 9, oneGbps] [9, 10, oneGbps] [10, 11, oneGbps] [11, 12, oneGbps] [6, 10, oneGbps] [7, 11, oneGbps] [8, 12, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 6, halfGbps] [6, 4, halfGbps] [1, 5, halfGbps] [4, 5, halfGbps] [7, 9, halfGbps] [8, 12, halfGbps] [12, 10, halfGbps] [7, 11, halfGbps] [10, 11, halfGbps] [1, 7, halfGbps] [2, 8, halfGbps] [9, 3, halfGbps] [10, 4, halfGbps]]'

push!(topologies, Topology("grid", nodesMatrix, edgesMatrix, trafficMatrix))

## Debugging.
# Very simple topologies, only for test purposes.
nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [2, 3, oneGbps] [3, 1, oneGbps]]'
trafficMatrix = [[1, 3, halfGbps] [2, 3, halfGbps]]'

push!(topologies, Topology("debug3", nodesMatrix, edgesMatrix, trafficMatrix))

nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [2, 3, oneGbps] [2, 1, oneGbps] [1, 3, oneGbps]]' # Supposed to be used with exactly these orientations.
trafficMatrix = [[1, 3, halfGbps] [2, 3, halfGbps]]'

push!(topologies, Topology("debug4", nodesMatrix, edgesMatrix, trafficMatrix))

nodesMatrix = permutedims([[1, "A"] [2, "B"]], [2, 1])
edgesMatrix = zeros(Int, 1, 3)
edgesMatrix[1, :] = [1, 2, oneGbps]
trafficMatrix = zeros(Int, 1, 3)
trafficMatrix[1, :] = [1, 2, halfGbps]

push!(topologies, Topology("debug", nodesMatrix, edgesMatrix, trafficMatrix))

nodesMatrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"] [7, "H"] [8, "I"] [9, "J"] [10, "K"] [11, "L"] [12, "M"]], [2, 1])
edgesMatrix = [[1, 2, oneGbps] [1, 3, oneGbps] [2, 4, oneGbps] [2, 5, oneGbps] [3, 6, oneGbps] [7, 8, oneGbps] [7, 9, oneGbps] [8, 10, oneGbps] [8, 11, oneGbps] [9, 12, oneGbps] [1, 7, oneGbps] [4, 10, oneGbps] [5, 11, oneGbps] [6, 12, oneGbps]]'
trafficMatrix = [[2, 8, halfGbps] [9, 2, halfGbps]]'

push!(topologies, Topology("debug_double_tree-connected", nodesMatrix, edgesMatrix, trafficMatrix))

### Finally, generate a list of names
names = sort(collect(t.name for t in topologies))
