using Common
using FileManager
using Visualization
using Detection
using AlphaStructures

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/CASALETTO"
INPUT_PC = FileManager.source2pc(source,0)
centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "CASALETTO"
folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"

folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
PC = FileManager.source2pc(joinpath(folders[3],"full_inliers.las"),1)

points = PC.coordinates
plane = Plane(points)
V = Common.apply_matrix(plane.matrix,points)[1:2,:]
DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = Common.estimate_threshold(V,30)
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
EV_boundary = Common.get_boundary_edges(V,FV)
w,EW = Lar.simplifyCells(V,EV_boundary)
W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))
model = (W,EW)

# GL.VIEW([
# 	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W))),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],0.8),
# ])


V, EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)
Detection.flushprint("Saves $(length(EV)) edges....")

GL.VIEW([
	#	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[rand(1:12)],0.8),
])
