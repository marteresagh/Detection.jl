println("loading packages... ")

using ArgParse
using Detection

println("packages OK")

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
		"source"
			help = "Input file LAS"
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
		"--thr"
            help = "threshold"
            arg_type = Float64
			required = true
    	"--failed"
			help = "number of failed before exit"
			arg_type = Int64
			default = 100
		"--validity"
			help = "number of points in a line"
			arg_type = Int64
			default = 100
		"--k"
			help = "number of neighbors"
			arg_type = Int64
			default = 10
		"--plane"
			help = "a, b, c, d parameters described the plane"
			arg_type = String
			required = true
    end

    return parse_args(s)
end

function main()
    args = parse_commandline()


	Detection.flushprintln("== params ==")
    for (arg,val) in args
        Detection.flushprintln("$arg  =>  $val")
    end

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	par = args["par"]
	threshold = args["thr"]
	failed = args["failed"]
	N = args["validity"]
	k = args["k"]
	plane = args["plane"]

	b = tryparse.(Float64,split(plane, " "))
	@assert length(b) == 4
		plane = Detection.Plane(b[1],b[2],b[3],b[4]])
	end
	affine_matrix = plane.matrix

	Detection.detection_and_saves(output_folder, project_name, source, par, threshold, failed, N, k, affine_matrix	)
end

@time main()
