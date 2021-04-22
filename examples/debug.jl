using Common
using FileManager
using Visualization
using Detection
using AlphaStructures


file = "C:/Users/marte/Documents/GEOWEB/TEST\\NAVVIS_LOD4\\plane_1283\\full_inliers.las"
file = "C:/Users/marte/Documents/GEOWEB/TEST\\NAVVIS_LOD4\\plane_1657\\full_inliers.las"
file = "C:/Users/marte/Documents/GEOWEB/TEST\\MURI_FULL\\plane_63783370976493\\full_inliers.las"
PC = FileManager.las2pointcloud(file)
points = PC.coordinates
plane = Plane(points)
V = Common.apply_matrix(plane.matrix,points)[1:2,:]
DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = Common.estimate_threshold(V,40)
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
EV_boundary = Common.get_boundary_edges(V,FV)
w,EW = Lar.simplifyCells(V,EV_boundary)

GL.VIEW([
	GL.GLGrid(w,EW)
])


GL.VIEW([
	[GL.GLGrid(w,EW[comp], GL.COLORS[rand(1:12)]) for comp in conn_comps]...
])

model = (w,EW)
w_,ew_ = Detection.simplify_model(model; par = 0.01, angle = pi/8)
GL.VIEW([
	GL.GLGrid(w_,ew_)
])

graph = Common.model2graph_edge2edge(w_,ew_)
# conn_comps = LightGraphs.connected_components(graph)
conn_comps = cycle(w_,ew_)
GL.VIEW([
	GL.GLGrid(w_,ew_[conn_comps[3]])
])

w3D_ = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w_,zeros(size(w_,2))'))
