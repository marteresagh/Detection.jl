"""
	pc2lines(
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
	# params = Initializer(PC, par, failed, N, k, affine_matrix; masterseeds = masterseeds)

	# POINTCLOUDS/FULL
	flushprintln("Slice: $(PC.n_points) points in slice")
	FileManager.save_pointcloud(joinpath(dirs.FULL,"slice.las"), PC, "VECTORIZATION" )

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

	# qui devo aprire segment 2d e segment 3d
	s_2d = open(joinpath(dirs.RAW,"segment2D.ext"), "w")
	s_3d = open(joinpath(dirs.RAW,"segment3D.ext"), "w")

	i = Detection.iterate_lines_detection(params, Lar.inv(affine_matrix), s_2d, s_3d; seeds = seeds)

	close(s_2d)
	close(s_3d)

	flushprintln("Detect $i lines")
	FileManager.successful(i!=0, dirs.output_folder)

	# 3. Saves
	flushprintln()
	flushprintln("=========== SAVES =============")

	save_partitions(PC, params, Lar.inv(affine_matrix), dirs)

end



"""
	pc2plane(
		folder::String,
		project_name::String,
		PC::PointCloud,
		par::Float64,
		failed::Int64,
		N::Int64,
		k::Int64;
		masterseeds = nothing::Union{String,Nothing}
		)

Main program.
Detect hyperplanes in point cloud.
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
