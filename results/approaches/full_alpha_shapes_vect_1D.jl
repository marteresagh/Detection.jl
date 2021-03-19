using Common
using FileManager
using Visualization
using Detection
using AlphaStructures

folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"
NAME_PROJ = "MURI_LOD3"
folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
boundary, full_boundary = FileManager.get_boundary(folders)
centroid = [291250.5043433152, 4.630341344699344e6, 106.74835850440863]

model = full_boundary[1] #11 12 16 17 20 28
V, EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,model[1])[1:2,:])')),
#	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Plane(V).matrix,V)[1:2,:],EV, GL.COLORS[2],1.)
])

#############################################################################

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "CASALETTO"
folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"

folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
PC = FileManager.source2pc(joinpath(folders[10],"full_inliers.las"),1)

points = PC.coordinates
plane = Plane(points)
V = Common.apply_matrix(plane.matrix,points)[1:2,:]

DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = Common.estimate_threshold(V,40)
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
EV_boundary = Common.get_boundary_edges(V,FV)
w,EW = Lar.simplifyCells(V,EV_boundary)
W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))
model = (W,EW)

V, EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)

# model = boundary_models[2]
GL.VIEW([
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],0.8),
])


GL.VIEW([
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],0.8),
])


##############################################################################
# GL.VIEW([
# 	GL.GLPoints(convert(Lar.Points,P')),
# 	[GL.GLPoints(convert(Lar.Points,dict[i]'), GL.COLORS[rand(1:12)]) for i in keys(dict)]...,
# #	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
# 	GL.GLGrid(P,EV, GL.COLORS[2],1.)
# ])
#
#
# #
# vect2D = Lar.LAR[]
# for i in 1:length(full_boundary)
# 	@show i
# 	model = full_boundary[i]
# 	V,EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)
# 	@show size(V)
# 	push!(vect2D,(V,EV))
# 	GL.VIEW([
# 		GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,model[1])[1:2,:])')),
# 		GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
# 		GL.GLGrid(Common.apply_matrix(Plane(V).matrix,V)[1:2,:],EV, GL.COLORS[2],1.)
# 	])
# end
#
# GL.VIEW(
# 	[GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),model[1]),model[2], GL.COLORS[rand(1:12)], 1.) for model in vect2D]
# )
