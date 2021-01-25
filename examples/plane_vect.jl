using Detection
using Visualization
using Common
using AlphaStructures
using FileManager

NAME_PROJ = "PLANE_NAVVIS"

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
	@show size(V)
	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	threshold = Common.estimate_threshold(V,10)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
end

#############################################

dirs = Detection.PlaneDirs( "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS",NAME_PROJ)

hyperplanes = Hyperplane[]
for (root, dirs, files) in walkdir(dirs.PLANE)
	for file in files
	 	push!(hyperplanes,FileManager.load_hyperplane(joinpath(root,file)))
	end
end

function DrawPlanes(plane::Hyperplane, AABB::Union{AABB,Nothing})
	DrawPlanes([plane], AABB)
end

function DrawPlanes(planes::Array{Hyperplane,1}, AABB::Union{AABB,Nothing})
	out = Array{Lar.Struct,1}()
	bb = deepcopy(AABB)
	for obj in planes
		plane = Plane(obj.direction,obj.centroid)
		points = obj.inliers.coordinates
		if isnothing(AABB)
			bb = Common.boundingbox(points)
		end
		points_flat = Common.apply_matrix(plane.matrix,points)
		extrema_x = extrema(points_flat[1,:])
		extrema_y = extrema(points_flat[2,:])
		extrema_z = extrema(points_flat[3,:])
		Vol = Volume([extrema_x[2]-extrema_x[1],extrema_y[2]-extrema_y[1],extrema_z[2]-extrema_z[1]],obj.centroid,Common.matrix2euler(Lar.inv(plane.matrix)))
		#triangulate vertex projected in plane XY
		V,EV,FV = getmodel(Vol)
		# FV = Common.delaunay_triangulation(V[1:2,:])
		cell = (V,sort.(FV))
		push!(out, Lar.Struct([cell]))
	end
	out = Lar.Struct( out )
	V,FV = Lar.struct2lar(out)
	return V,FV
end


V,FV = DrawPlanes(hyperplanes, nothing)
centroid = Common.centroid(V)

GL.VIEW([
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])

W,EW = save_alpha_shape_model(hyperplanes, "")
W2 = FileManager.load_points(joinpath(dirs.A_SHAPES,"a_shapes_points.txt"))
EW2 = FileManager.load_cells(joinpath(dirs.A_SHAPES,"a_shapes_edges.txt"))

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),W)'),GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
	# GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W2),EW2,GL.COLORS[2],1.0),
	#Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])
