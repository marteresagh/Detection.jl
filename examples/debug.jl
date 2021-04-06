using Common
using FileManager
using Visualization
using Detection

W = FileManager.load_points("points.txt")
EW = FileManager.load_cells("edges.txt")

GL.VIEW([
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W))),
	GL.GLGrid(W,EW,GL.COLORS[1],0.8),
	GL.GLFrame2
])

###################### debug
graph = Common.model2graph_edge2edge(W,EW)
conn_comps = LightGraphs.connected_components(graph)
subgraph = LightGraphs.induced_subgraph(graph, conn_comps[1])
# estraggo catene lineari come collezioni di indici
all_clusters_in_model = Array{Array{Int64,1},1}[]
cl_edges = Detection.get_cluster_edges((W,EW), subgraph; par = 0.02, angle = pi/8)
push!(all_clusters_in_model, cl_edges)

GL.VIEW([
	GL.GLGrid(W,EW,GL.COLORS[1],0.8),
	[GL.GLGrid(W,EW[cl_edges[i]],GL.COLORS[rand(1:12)],0.8) for i in 1:length(cl_edges)]...,
])


cluss = union(all_clusters_in_model...)

# costruisco i nuovi spigoli eliminando i punti interni della catena costruita
EV, dict = Detection.simplify_edges(EV, cluss, dict)

# calcolo nuovi punti di intersezione (e modifico P forse non devo modificare P?)
# optimize!(P, EV, dict)

# rimuovo alcuni spigoli non necessari
EV, dict  = remove_some_edges!(P, EV, dict; par = par, angle = angle)  


V, EV, dict = Detection.simplify_model(model; par = 0.02, angle = pi/8)

plane = Plane(V)
V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
W2D = Common.apply_matrix(plane.matrix,W)[1:2,:]
PC2D = Common.apply_matrix(plane.matrix,PC.coordinates)[1:2,:]
# FileManager.save_connected_components("prova.txt",V,EV)
# EV = FileManager.load_connected_components("prova.txt")
GL.VIEW([

	# GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),PC2D))),
	# GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D),EV[[5]],GL.COLORS[1],0.8),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),W2D),EW,GL.COLORS[2],0.8),
])

GL.VIEW([
	[GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),dict[k])), GL.COLORS[rand(1:12)]) for k in keys(dict)]...,
])
# #
# p = dict[1]
# params = Common.LinearFit(p)
# hyperplane = Hyperplane(PointCloud(p),params...)
# Z,EZ = Common.DrawLines(hyperplane)
# GL.VIEW([
# 	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(p)...),W2D))),
# 	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(p)...),p)'),GL.RED),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(p)...),Z),EZ,GL.COLORS[1],1.0)
# ])
