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
	println("=========== INIT =============")
	# output directory
	output_folder = FileManager.mkdir_project(folder,project_name)

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
	i = iterate_planes_detection(params, output_folder; seeds = seeds)

	# 3. Saves
	println()
	println("=========== RESULTS =============")
	println("$i planes detected")
	FileManager.successful(i!=0, output_folder; filename = "vectorize_2D.probe")

end
