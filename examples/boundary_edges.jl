using Detection
using Visualization
using Common
using AlphaStructures
using FileManager


function get_boundary_alpha_shape(hyperplane::Hyperplane,plane::Plane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
end

function boundary_shapes(hyperplanes::Array{Hyperplane,1}, threshold::Float64)::Lar.LAR
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		input_model = get_boundary_alpha_shape(hyperplane,plane)

		models = Detection.get_linerized_models(input_model)

		for model in models
			vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
			out = push!(out, Lar.Struct([(vertices, model[2])]))
		end

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)

	# 5. salvo il modello come??

	return V,EV
end

W = FileManager.load_points("point.txt")
EW = FileManager.load_cells("edges.txt")
GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],1.0)])
