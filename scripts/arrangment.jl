println("loading packages... ")

using ArgParse
using Detection
using Common

println("packages OK")

# function boundary_shape

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"--projectname", "-p"
		help = "Project name"
		required = true
	"--output", "-o"
		help = "Output folder"
		required = true
	"--par"
		help = "Parameter"
		arg_type = Float64
		default = 0.02
	"--angle"
		help = "Angle"
		arg_type = Float64
		default = pi/8
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 40
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	project_name = args["projectname"]
	output_folder = args["output"]
	par = args["par"]
	angle = args["angle"]
	k = args["k"]

	println("== Parameters ==")
	println("Output folder  =>  $output_folder")
	println("Project name  =>  $project_name")
	println("Parameter  =>  $par")
	println("Angle  =>  $angle")

	flush(stdout)

	folders = Detection.get_plane_folders(output_folder, project_name)


end

@time main()
