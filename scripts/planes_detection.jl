println("loading packages...")

using ArgParse
using Detection
using FileManager
using Search

println("packages OK")

# function boundary_shape

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Input Potree"
		required = true
	"--projectname", "-p"
		help = "Project name"
		required = true
	"--output", "-o"
		help = "Output folder"
		required = true
	"--par"
		help = "Parameter"
		arg_type = Float64
		required = true
	"--lod"
		help = "Level of detail. If -1, all points are taken"
		arg_type = Int64
		default = 1
	"--failed"
		help = "number of failed before exit"
		arg_type = Int64
		default = 100
	"--validity"
		help = "number of points in a line"
		arg_type = Int64
		default = 40
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 30
	"--masterseeds","-s"
		help = "A text file with seeds list"
		arg_type = String
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	par = args["par"]
	failed = args["failed"]
	N = args["validity"]
	k = args["k"]
	lod = args["lod"]
	masterseeds = args["masterseeds"]

	# input point cloud
	PC = FileManager.source2pc(source, lod)

	println("== Parameters ==")
	println("Source  =>  $source")
	println("Output folder  =>  $output_folder")
	println("Project name  =>  $project_name")
	println("Parameter  =>  $par")
	println("Seeds =>  $(args["masterseeds"])")
	println("N. of failed  =>  $failed")
	println("N. of inliers  =>  $N")
	println("N. of k-nn  =>  $k")

	flush(stdout)

	println("=========== INIT =============")
	# output directory
	project_folder = FileManager.mkdir_project(output_folder,project_name)

	# seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		println("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Search.consistent_seeds(PC).([c[:] for c in eachcol(given_seeds)])
	end

	params = Detection.Initializer(PC, par, failed,	N, k)

	# 2. Detection
	println()
	println("=========== PROCESSING =============")
	i = Detection.iterate_planes_detection(params, project_folder; seeds = seeds)

	# 3. Saves
	println()
	println("=========== RESULTS =============")
	println("$i planes detected")
	FileManager.successful(i!=0, project_folder; filename = "vectorize_2D.probe")
end

@time main()
