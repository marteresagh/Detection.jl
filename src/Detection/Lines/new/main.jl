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
function vect1D(
	folder::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64,
	affine_matrix::Matrix;
	masterseeds = nothing::Union{String,Nothing}
	)

	# 1. Initialization
	println()
	println("=========== INIT =============")

	# output directory
	dirs = Vect_1D_Dirs(folder, "VECT")

	# POINTCLOUDS/FULL
	println("Slice: $(PC.n_points) points in slice")
	FileManager.save_pointcloud(joinpath(dirs.FULL,"slice.las"), PC, "VECTORIZATION_1D" )

	INPUT_PC = PointCloud(Common.apply_matrix(affine_matrix,PC.coordinates)[1:2,:], PC.rgbs)

	params = Initializer(INPUT_PC, par, failed, N, k)

	#seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		# seeds indices
		println("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		given_seeds_2D = Common.apply_matrix(affine_matrix,given_seeds)[1:2,:]
		seeds = Search.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds_2D)])
	end

	# 2. Detection
	println()
	println("=========== PROCESSING =============")

	# qui devo aprire segment 2d e segment 3d
	s_2d = open(joinpath(dirs.RAW,"segment2D.ext"), "w")
	s_3d = open(joinpath(dirs.RAW,"segment3D.ext"), "w")

	i = iterate_lines_detection(params, Common.inv(affine_matrix), s_2d, s_3d; seeds = seeds)

	close(s_2d)
	close(s_3d)

	# 3. Saves
	println()
	println("=========== RESULTS =============")
	println("$i lines detected")
	params.PC = Common.PointCloud()
	params.lines = i
	save_partitions(PC, params, Common.inv(affine_matrix), dirs)
	return params
end
