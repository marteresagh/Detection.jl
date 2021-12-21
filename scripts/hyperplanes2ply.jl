println("loading packages...")

using ArgParse
using Detection

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
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()
	project_name = args["projectname"]
	output_folder = args["output"]

	println("== Parameters ==")
	println("Output folder  =>  $output_folder")
	println("Project name  =>  $project_name")

	flush(stdout)
	folders = Detection.get_plane_folders(output_folder,project_name)
	hyperplanes, _ = Detection.get_hyperplanes(folders)
	Detection.save_plane_segments_in_ply(hyperplanes, joinpath(output_folder,"segments.ply"))
end

@time main()
