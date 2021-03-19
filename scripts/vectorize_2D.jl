println("loading packages... ")

using ArgParse
using Detection
using Common

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
	PC = Detection.FileManager.source2pc(source, lod)

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("Seeds =>  $(args["masterseeds"])")
	Detection.flushprintln("N. of failed  =>  $failed")
	Detection.flushprintln("N. of inliers  =>  $N")
	Detection.flushprintln("N. of k-nn  =>  $k")

	# detection
	Detection.pc2plane(output_folder, project_name, PC, par, failed, N, k; masterseeds = masterseeds)

end

@time main()
