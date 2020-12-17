println("loading packages... ")

using ArgParse
using Detection
using Common
using AlphaStructures

println("packages OK")

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
		default = 5
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 20
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

	# input point cloud
	PC = Detection.FileManager.source2pc(source, lod)

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Threshold =>  $threshold")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("N. of failed  =>  $failed")
	Detection.flushprintln("N. of points on line  =>  $N")
	Detection.flushprintln("N. of k-nn  =>  $k")
	Detection.flushprintln("Affine matrix =>  $affine_matrix")

	# detection
	Detection.flushprintln(" ")
	Detection.flushprintln("=== Planes detection ===")

	hyperplanes, params, dirs = Detection.pc2vectorize(output_folder, project_name, PC, par, failed, N, k, affine_matrix, false)

	# vectorized
	Detection.flushprintln(" ")
	Detection.flushprintln("=== Get boundary shapes ===")

	filename = joinpath(dirs.output_folder,"$(project_name)_vectorize_2D.txt")
	V,EV = Detection.get_boundary_shapes(filename::String, hyperplanes::Array{Hyperplanes,1})

	# process boundary edges
	Detection.linearization(V,EV)

end

@time main()
