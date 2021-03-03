#
# # alpha shape
# function save_alpha_shape_model(hyperplanes::Array{Hyperplane,1})
# 	out = Array{Lar.Struct,1}()
# 	for i in 1:length(hyperplanes)
#
# 		Detection.flushprintln("$i planes processed")
#
# 		hyperplane = hyperplanes[i]
# 		plane = Plane(hyperplane.direction, hyperplane.centroid)
#
# 		model = get_boundary_alpha_shape(hyperplane,plane)
#
# 		vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
# 		out = push!(out, Lar.Struct([(vertices, model[2])]))
#
# 	end
# 	out = Lar.Struct(out)
# 	V,EV = Lar.struct2lar(out)
# 	FileManager.save_points_txt("a_shapes_points.txt", V)
# 	FileManager.save_cells_txt("a_shapes_edges.txt", EV)
# 	return V,EV
# end
#
# function get_boundary_alpha_shape(hyperplane::Hyperplane, plane::Plane)
# 	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
# 	points = hyperplane.inliers.coordinates
# 	V = Common.apply_matrix(plane.matrix,points)[1:2,:]
#
# 	# 2. applica alpha shape con alpha = threshold
# 	filtration = AlphaStructures.alphaFilter(V);
# 	threshold = Common.estimate_threshold(hyperplane.inliers,5)
# 	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
#
# 	# 3. estrai bordo
# 	EV_boundary = Common.get_boundary_edges(V,FV)
# 	return Lar.simplifyCells(V,EV_boundary)
# end
