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
Detect lines in a thin slice of point cloud.
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
	flushprintln()
	flushprintln("=========== INIT =============")

	# output directory
	dirs = VectDirs(folder, project_name)

	# POINTCLOUDS/FULL
	flushprintln("Slice: $(PC.n_points) points in slice")
	FileManager.save_pointcloud(joinpath(dirs.FULL,"slice.las"), PC, "VECTORIZATION_1D" )

	INPUT_PC = PointCloud(Common.apply_matrix(affine_matrix,PC.coordinates)[1:2,:], PC.rgbs)

	params = Initializer(INPUT_PC, par, failed, N, k)

	#seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		# seeds indices
		flushprintln("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		given_seeds_2D = Common.apply_matrix(affine_matrix,given_seeds)[1:2,:]
		seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds_2D)])
	end

	# 2. Detection
	flushprintln()
	flushprintln("=========== PROCESSING =============")

	# qui devo aprire segment 2d e segment 3d
	s_2d = open(joinpath(dirs.RAW,"segment2D.ext"), "w")
	s_3d = open(joinpath(dirs.RAW,"segment3D.ext"), "w")

	i = Detection.iterate_lines_detection(params, Lar.inv(affine_matrix), s_2d, s_3d; seeds = seeds)

	close(s_2d)
	close(s_3d)

	# 3. Saves
	flushprintln()
	flushprintln("=========== RESULTS =============")
	flushprintln("$i lines detected")
	FileManager.successful(i!=0, dirs.output_folder)
	save_partitions(PC, params, Lar.inv(affine_matrix), dirs)

end
