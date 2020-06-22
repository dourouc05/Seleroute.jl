using Seleroute, JuMP, LightGraphs, CPLEX
using Profile

nodes_matrix = permutedims([[1, "A"] [2, "B"] [3, "C"] [4, "D"] [5, "E"] [6, "F"]], [2, 1])
edges_matrix = [[1, 2, 1] [2, 3, .9] [2, 6, 1] [3, 4, 1] [3, 6, 1] [5, 6, 1]]'
traffic_matrix = Float64[[1, 3, 0.5] [1, 6, 0.5] [4, 1, 0.5] [4, 5, 0.5] [5, 2, 0.5] [6, 4, 0.5]]'
traffic_matrix[:, 3] .*= 0.95

opt = CPLEX.Optimizer
t = Topology("basic", nodes_matrix, edges_matrix, traffic_matrix)
g, k, dm = topology_to_graphs(t)
mt = ModelType(Load(), MinMaxFair())
rd = RoutingData(g, k, opt, mt, name=t.name, traffic_matrix=dm)

# First dimension: Edge{Int64}[Edge 1 => 3, Edge 1 => 6, Edge 4 => 1, Edge 4 => 5, Edge 5 => 2, Edge 6 => 4]
# Second dimension: Edge{Int64}[Edge 1 => 2, Edge 2 => 1, Edge 2 => 3, Edge 2 => 6, Edge 3 => 2, Edge 3 => 4, Edge 3 => 6, Edge 4 => 3, Edge 5 => 6, Edge 6 => 2, Edge 6 => 3, Edge 6 => 5]
# 1.0                   0.0                   0.9999988912829403     2.120611460099956e-6  0.0                  0.0                   0.0                    0.0                   1.1094507307655902e-8  1.0122885971469296e-6  1.1087170597439012e-6  1.1094507307655902e-8
# 1.0                   0.0                   0.3177048579857707     0.7726791282105232    0.09038398659069616  2.476831093836052e-7  0.22732087178947685    2.476831093836052e-7  0.0                    0.0                    0.0                    0.0
# 0.0                   1.0                   1.2410080677635113e-6  8.844169471686035e-7  0.9999987504657342   0.0                   3.3707368767873062e-6  1.0                   0.0                    3.374629109674882e-6   8.805247142810213e-7   0.0
# 3.087618250488776e-7  3.087618250488776e-7  0.08955509114375726    0.2343737049053851    0.3239280973532021   0.0                   0.76562769985842       1.0                   0.0                    6.986957449624771e-7   7.063964009965341e-7   1.0
# 0.0                   0.0                   0.0                    0.0                   0.2126523775599113   0.0                   6.99368822994739e-7    0.0                   1.0                    0.7873476224400887     0.2126530769949393     0.0
# 0.0                   0.0                   0.3037925494376403     0.0                   0.09114078583113316  1.0                   0.0                    0.0                   0.0                    0.21265176354084359    0.7873482364591564     0.0
sol = JuMP.Containers.DenseAxisArray{Float64}(undef, edges(k), edges(g))
sol[Edge(1, 3), Edge(1, 2)] = 1.0
sol[Edge(1, 3), Edge(2, 1)] = 0.0
sol[Edge(1, 3), Edge(2, 3)] = 0.9999988912829403
sol[Edge(1, 3), Edge(2, 6)] = 2.120611460099956e-6
sol[Edge(1, 3), Edge(3, 2)] = 0.0
sol[Edge(1, 3), Edge(3, 4)] = 0.0
sol[Edge(1, 3), Edge(3, 6)] = 0.0
sol[Edge(1, 3), Edge(4, 3)] = 0.0
sol[Edge(1, 3), Edge(5, 6)] = 1.1094507307655902e-8
sol[Edge(1, 3), Edge(6, 2)] = 1.0122885971469296e-6
sol[Edge(1, 3), Edge(6, 3)] = 1.1087170597439012e-6
sol[Edge(1, 3), Edge(6, 5)] = 1.1094507307655902e-8

sol[Edge(1, 6), Edge(1, 2)] = 1.0
sol[Edge(1, 6), Edge(2, 1)] = 0.0
sol[Edge(1, 6), Edge(2, 3)] = 0.3177048579857707
sol[Edge(1, 6), Edge(2, 6)] = 0.7726791282105232
sol[Edge(1, 6), Edge(3, 2)] = 0.09038398659069616
sol[Edge(1, 6), Edge(3, 4)] = 2.476831093836052e-7
sol[Edge(1, 6), Edge(3, 6)] = 0.22732087178947685
sol[Edge(1, 6), Edge(4, 3)] = 2.476831093836052e-7
sol[Edge(1, 6), Edge(5, 6)] = 0.0
sol[Edge(1, 6), Edge(6, 2)] = 0.0
sol[Edge(1, 6), Edge(6, 3)] = 0.0
sol[Edge(1, 6), Edge(6, 5)] = 0.0

sol[Edge(4, 1), Edge(1, 2)] = 0.0
sol[Edge(4, 1), Edge(2, 1)] = 1.0
sol[Edge(4, 1), Edge(2, 3)] = 1.2410080677635113e-6
sol[Edge(4, 1), Edge(2, 6)] = 8.844169471686035e-7
sol[Edge(4, 1), Edge(3, 2)] = 0.9999987504657342
sol[Edge(4, 1), Edge(3, 4)] = 0.0
sol[Edge(4, 1), Edge(3, 6)] = 3.3707368767873062e-6
sol[Edge(4, 1), Edge(4, 3)] = 1.0
sol[Edge(4, 1), Edge(5, 6)] = 0.0
sol[Edge(4, 1), Edge(6, 2)] = 3.374629109674882e-6
sol[Edge(4, 1), Edge(6, 3)] = 8.805247142810213e-7
sol[Edge(4, 1), Edge(6, 5)] = 0.0

sol[Edge(4, 5), Edge(1, 2)] = 3.087618250488776e-7
sol[Edge(4, 5), Edge(2, 1)] = 3.087618250488776e-7
sol[Edge(4, 5), Edge(2, 3)] = 0.08955509114375726
sol[Edge(4, 5), Edge(2, 6)] = 0.2343737049053851
sol[Edge(4, 5), Edge(3, 2)] = 0.3239280973532021
sol[Edge(4, 5), Edge(3, 4)] = 0.0
sol[Edge(4, 5), Edge(3, 6)] = 0.76562769985842
sol[Edge(4, 5), Edge(4, 3)] = 1.0
sol[Edge(4, 5), Edge(5, 6)] = 0.0
sol[Edge(4, 5), Edge(6, 2)] = 6.986957449624771e-7
sol[Edge(4, 5), Edge(6, 3)] = 7.063964009965341e-7
sol[Edge(4, 5), Edge(6, 5)] = 1.0

sol[Edge(5, 2), Edge(1, 2)] = 0.0
sol[Edge(5, 2), Edge(2, 1)] = 0.0
sol[Edge(5, 2), Edge(2, 3)] = 0.0
sol[Edge(5, 2), Edge(2, 6)] = 0.0
sol[Edge(5, 2), Edge(3, 2)] = 0.2126523775599113
sol[Edge(5, 2), Edge(3, 4)] = 0.0
sol[Edge(5, 2), Edge(3, 6)] = 6.99368822994739e-7
sol[Edge(5, 2), Edge(4, 3)] = 0.0
sol[Edge(5, 2), Edge(5, 6)] = 1.0
sol[Edge(5, 2), Edge(6, 2)] = 0.7873476224400887
sol[Edge(5, 2), Edge(6, 3)] = 0.2126530769949393
sol[Edge(5, 2), Edge(6, 5)] = 0.0

sol[Edge(6, 4), Edge(1, 2)] = 0.0
sol[Edge(6, 4), Edge(2, 1)] = 0.0
sol[Edge(6, 4), Edge(2, 3)] = 0.3037925494376403
sol[Edge(6, 4), Edge(2, 6)] = 0.0
sol[Edge(6, 4), Edge(3, 2)] = 0.09114078583113316
sol[Edge(6, 4), Edge(3, 4)] = 1.0
sol[Edge(6, 4), Edge(3, 6)] = 0.0
sol[Edge(6, 4), Edge(4, 3)] = 0.0
sol[Edge(6, 4), Edge(5, 6)] = 0.0
sol[Edge(6, 4), Edge(6, 2)] = 0.21265176354084359
sol[Edge(6, 4), Edge(6, 3)] = 0.7873482364591564
sol[Edge(6, 4), Edge(6, 5)] = 0.0

println("Starting profiling")
@profile flow_routing_to_path(rd, sol)
