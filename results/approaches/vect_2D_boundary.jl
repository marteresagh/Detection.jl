using Common
using FileManager
using Visualization
using Detection
using AlphaStructures

function modello_bordo(PC::PointCloud)
	function outliers(points)
		outliers = Int64[]
		tree = Common.KDTree(points)
		idxs = Common.inrange(tree, points, 0.04)
		for i in 1:length(idxs)
			if length(idxs[i])<3
				push!(outliers,i)
			end
		end
		return outliers
	end
	points = PC.coordinates
	plane = Plane(points)
	T = Common.apply_matrix(plane.matrix,points)[1:2,:]

	GL.VIEW([
		GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(T)...),T))),
	])


	out = outliers(T) #Common.outliers(PC, collect(1:PC.n_points), 30)
	V = T[:, setdiff( collect(1:PC.n_points), out)]

	GL.VIEW([
		GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(T)...),T))),
		GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(T)...),T[:,out])), GL.COLORS[2]),
	])

	DT = Common.delaunay_triangulation(V)
	filtration = AlphaStructures.alphaFilter(V,DT);
	threshold = Common.estimate_threshold(V,30)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
	EV_boundary = Common.get_boundary_edges(V,FV)
	w,EW = Lar.simplifyCells(V,EV_boundary)
	W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))
	model = (W,EW)
	return model
end


source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,0)
centroid = Common.centroid(INPUT_PC.coordinates)

folder = "C:/Users/marte/Documents/GEOWEB/TEST/MURI_FULL/plane_63783370976493"
PC = FileManager.source2pc(joinpath(folder,"full_inliers.las"),0)


model = modello_bordo(PC)
W,EW = model

GL.VIEW([
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],0.8),
])

#
V, EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)

plane = Plane(V)
V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
W2D = Common.apply_matrix(plane.matrix,W)[1:2,:]
# FileManager.save_connected_components("prova.txt",V,EV)
# EV = FileManager.load_connected_components("prova.txt")
GL.VIEW([
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),W2D))),
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D),EV,GL.COLORS[1],0.8),
])

#######################################  DEBUG ###################################################
V,EV,dict = simplify_model(model; par = 0.01, angle = pi/8)
plane = Plane(V)
V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
W2D = Common.apply_matrix(plane.matrix,W)[1:2,:]
FileManager.save_connected_components("prova.txt",V,EV)
EV = FileManager.load_connected_components("prova.txt")
GL.VIEW([
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),W2D))),
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),V2D),EV,GL.COLORS[1],0.8),
])

# FileManager.save_connected_components("prova.txt",V,EV)
# EV = FileManager.load_connected_components("prova.txt")
GL.VIEW([
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(P)...),W2D))),
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(P)...),P))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(P)...),P),EV,GL.COLORS[1],0.8),
])

GL.VIEW([
	[GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),dict[i])), GL.COLORS[i-12]) for i in 13:24]...,
])
