"""
	pc2vectorize(
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

Main program.
Detect hyperplanes in point cloud. In 2D space results are saved in different files.
"""
function pc2lines(
	folder::String,
	project_name::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64,
	affine_matrix::Matrix;
	masterseeds = nothing::Union{String,Nothing}
	)

	# 1. Initialization
	flushprintln("=========== INIT =============")

	# output directory
	dirs = VectDirs(folder, project_name)

	INPUT_PC = PointCloud(Common.apply_matrix(affine_matrix,PC.coordinates)[1:2,:], PC.rgbs)

	#seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		# seeds indices
		flushprintln("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		given_seeds_2D = Common.apply_matrix(affine_matrix,given_seeds)[1:2,:]
		seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds_2D)])
	end

	# threashold estimation
	threshold = Common.estimate_threshold(INPUT_PC,2*k)
	flushprintln("Compute threshold: $threshold")

	flushprintln("= Remove points from possible seeds =")
	flushprintln("Search of possible outliers: ")
	outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	flushprintln("$(length(outliers)) outliers")

	# flushprintln("Search of points with high curvature")
	# corners = Detection.corners_detection(INPUT_PC, par, threshold)
	# flushprintln("$(length(corners)) points on corners")

	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

	# 2. Detection
	flushprintln()
	flushprintln("=========== PROCESSING =============")
	hyperplanes = Detection.iterate_detection(params; seeds = seeds)

	# 3. Saves
	flushprintln()
	flushprintln("=========== SAVES =============")

	saves_data(PC, params, hyperplanes, Lar.inv(affine_matrix), dirs)

	return hyperplanes, params, dirs
end



"""
	pc2vectorize(
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

Main program.
Detect hyperplanes in point cloud. In 2D space results are saved in different files.
"""
function pc2plane(
	folder::String,
	project_name::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64;
	masterseeds = nothing::Union{String,Nothing}
	)

	# 1. Initialization
	flushprintln("=========== INIT =============")
	# output directory
	dirs = PlaneDirs(folder, project_name)

	INPUT_PC = PC

	# seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		flushprintln("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])
	end

	# threashold estimation
	threshold = Common.estimate_threshold(INPUT_PC,2*k)
	flushprintln("Compute threshold: $threshold")

	# normals
	flushprintln("Compute normals")
	INPUT_PC.normals = Common.compute_normals(INPUT_PC.coordinates,threshold,k)

	# 1. Initialization
	flushprintln("= Remove points from possible seeds =")
	flushprintln("Search of possible outliers: ")
	outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	flushprintln("$(length(outliers)) outliers")

	# flushprintln("Search of points with high curvature")
	# corners = Detection.corners_detection(INPUT_PC, par, threshold)
	# flushprintln("$(length(corners)) points on corners")

	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

	# 2. Detection
	flushprintln()
	flushprintln("=========== PROCESSING =============")
	hyperplanes = Detection.iterate_detection(params; seeds = seeds)

	# 3. Saves


	return hyperplanes, params, dirs
end
