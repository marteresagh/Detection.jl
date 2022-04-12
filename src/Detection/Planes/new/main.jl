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
function vect2D(
	folder::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64;
	masterseeds = nothing::Union{String,Nothing}
	)

	# 1. Initialization
	println("=========== INIT =============")

	# Input Point Cloud
	println("Pointcloud: $(PC.n_points) points")

	# seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		println("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Search.consistent_seeds(PC).([c[:] for c in eachcol(given_seeds)])
	end

	params = Initializer(PC, par, failed,	N, k)

	# 2. Detection
	println()
	println("=========== PROCESSING =============")
	i = iterate_planes_detection(params, folder; seeds = seeds)

	# 3. Saves
	println()
	println("=========== RESULTS =============")
	println("$i lines detected")
	params.PC = Common.PointCloud()
	params.hyperplanes = i
	return params
end
