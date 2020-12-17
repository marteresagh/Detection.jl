"""
Main
"""
function pc2vectorize(
	folder::String,
	project_name::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64,
	affine_matrix::Matrix,
	lines = true::Bool
	)


	flushprintln("=========== INIT =============")
	# output directory
	dirs = VectDirs(folder, project_name)

	if lines
		INPUT_PC = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	# 1. Initialization
	flushprintln("Search of possible outliers to remove: ")
	outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	flushprintln("$(length(outliers)) outliers")

	flushprintln()
	flushprintln("=========== PROCESSING =============")

	# threashold estimation
	threshold = Common.estimate_threshold(INPUT_PC,k)

	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

	# 2. Detection
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. Saves
	flushprintln()
	flushprintln("=========== SAVES =============")

	if lines
		saves_data(PC, params, hyperplanes, affine_matrix, dirs)
	end

	return hyperplanes, params, dirs
end

function get_boundary_shapes(filename::String, hyperplanes::Array{Hyperplanes,1})

	io = open(filename,"w")
	for i in 1:length(hyperplanes)

		hyperplane = hyperplanes[i]

		# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
		points = hyperplane.inliers.coordinates
		plane = Plane(hyperplane.direction..., Lar.dot(hyperplane.direction,hyperplane.centroid))
		T = Common.apply_matrix(Lar.inv(plane.matrix),points)[1:2,:]

		# 2. applica alpha shape con alpha = threshold
		filtration = AlphaStructures.alphaFilter(T);
		_, _, FV = AlphaStructures.alphaSimplex(T, filtration, threshold)

		# 3. estrai bordo
		EV_boundary = Common.get_boundary_edges(T,FV)

		# 4. salva i segmenti del bordo in 3D
		T = Common.points_projection_on_plane(points, hyperplane)
		for ev in EV_boundary
			write(io, "$(T[1,ev[1]]) $(T[2,ev[1]]) $(T[3,ev[1]]) $(T[1,ev[2]]) $(T[2,ev[2]]) $(T[3,ev[2]])/n")
		end

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end
	end

	close(io)
	return FileManager.load_segment(filename) # V,EV
end


function linearization(V::Lar.Points,EV::Lar.Cells)

end
