"""
	pc2vectorize(
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

Main program.
Detect hyperplanes in point cloud. In 2D space results are saved in different files.
"""
function pc2vectorize(
	folder::String,
	project_name::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64,
	affine_matrix::Matrix;
	masterseeds = nothing::Union{String,Nothing},
	lines = true::Bool
	)


	flushprintln("=========== INIT =============")
	# output directory
	dirs = VectDirs(folder, project_name)

	if lines
		INPUT_PC = PointCloud(Common.apply_matrix(affine_matrix,PC.coordinates)[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end


	if isnothing(masterseeds) # if seeds are not provided
		# 1. Initialization
		flushprintln("= Remove points from possible seeds =")
		flushprintln("Search of possible outliers: ")
		outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
		flushprintln("$(length(outliers)) outliers")

		# flushprintln("Search of points with high curvature")
		# corners = Detection.corners_detection(INPUT_PC, par, threshold)
		# flushprintln("$(length(corners)) points on corners")

		flushprintln()
		flushprintln("=========== PROCESSING =============")

		# threashold estimation
		threshold = Common.estimate_threshold(INPUT_PC,k)

		params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

		# 2. Detection
		hyperplanes = Detection.iterate_random_detection(params)
	else # if seeds are provided
		# 1. Initialization
		flushprintln()
		flushprintln("=========== PROCESSING =============")

		# threashold estimation
		threshold = Common.estimate_threshold(INPUT_PC,k)

		# seeds indices
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

		params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

		# 2. Detection
		hyperplanes = Detection.iterate_seeds_detection(params,seeds)

	end

	# 3. Saves
	if lines
		flushprintln()
		flushprintln("=========== SAVES =============")

		saves_data(PC, params, hyperplanes, Lar.inv(affine_matrix), dirs)
	end

	return hyperplanes, params, dirs
end
