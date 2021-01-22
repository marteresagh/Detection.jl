using Detection
using Visualization
using Common
using AlphaStructures
using FileManager


####################

function save_alpha_shape_model(hyperplanes::Array{Hyperplane,1}, name_proj::String)
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		Detection.flushprintln("$i planes processed")

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		model = get_boundary_alpha_shape(hyperplane,plane)

		vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
		out = push!(out, Lar.Struct([(vertices, model[2])]))

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)
	# FileManager.save_points_txt(name_proj*"_points.txt", V)
	# FileManager.save_cells_txt(name_proj*"_edges.txt", EV)
	return V,EV
end

function get_boundary_alpha_shape(hyperplane::Hyperplane, plane::Plane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	threshold = Common.estimate_threshold(hyperplane.inliers,5)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
end

#############################################

dirs = Detection.PlaneDirs( "C:/Users/marte/Documents/GEOWEB/TEST","PLANE_CONTEA")

hyperplanes = Hyperplane[]
for (root, dirs, files) in walkdir(dirs.PLANE)
	for file in files
	 	push!(hyperplanes,FileManager.load_hyperplane(joinpath(root,file)))
	end
end

V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)
centroid = Common.centroid(V)

GL.VIEW([
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])

W,EW = save_alpha_shape_model(hyperplanes, joinpath(dirs.A_SHAPES,"a_shapes"))
W = FileManager.load_points(joinpath(dirs.A_SHAPES,"a_shapes_points.txt"))
EW = FileManager.load_cells(joinpath(dirs.A_SHAPES,"a_shapes_edges.txt"))

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),W)'),GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
	#Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])
