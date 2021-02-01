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
	output_folder = FileManager.mkdir_project(folder,project_name)

	# Input Point Cloud
	INPUT_PC = PC

	# seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		flushprintln("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])
	end

	# normals
	flushprintln("Compute normals")
	INPUT_PC.normals = Common.compute_normals(INPUT_PC.coordinates,threshold,k)

	params = Initializer(INPUT_PC, par, failed,	N, k)

	# 2. Detection
	flushprintln()
	flushprintln("=========== PROCESSING =============")
	hyperplanes = Detection.iterate_planes_detection(params, output_folder; seeds = seeds)

	# 3. Saves


	return hyperplanes, params, dirs
end
