using Common
using Detection
using LightGraphs
using AlphaStructures
using Visualization

function model2graph(V::Lar.Points,EV::Lar.Cells)
	graph = SimpleGraph(size(V,2))
	for ev in EV
		add_edge!(graph,ev[1],ev[2])
	end
	return graph
end

V = rand(2,1000)
# 2. applica alpha shape con alpha = threshold
DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = 0.1
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

# 3. estrai bordo
EV_boundary = Common.get_boundary_edges(V,FV)
V,EV = Lar.simplifyCells(V,EV_boundary)

out = Lar.Struct([(V,EV),Lar.t(3,3),(V,EV)])
V,EV = Lar.struct2lar(out)

GL.VIEW([
	GL.GLGrid(V,EV,GL.COLORS[2],1.0),
])


g = model2graph(V,EV)
conn_comps = connected_components(g)

rows = []
for comp in conn_comps
	subgraph,vmap = induced_subgraph(g, comp)
	path = dfs_tree(subgraph, 1)
	edges = topological_sort_by_dfs(path)
	push!(rows,vmap[edges])
end


function generate_EV(path)
	EV=[[path[i],path[i+1]] for i in 1:length(path)-1]
	push!(EV,[path[end],path[1]])
end

EV = generate_EV(rows[1])

GL.VIEW([
	GL.GLGrid(V,EV,GL.COLORS[2],1.0),
	GL.GLFrame2
])



io = open("boundary_edges2.txt","w")
g = Common.model2graph(V,EV)
conn_comps = connected_components(g)
for comp in conn_comps
	@show "ciao"
	subgraph,vmap = induced_subgraph(g, comp)
	path = dfs_tree(subgraph, 1)
	edges = topological_sort_by_dfs(path)
	write(io,"$edges\n")
end
close(io)
