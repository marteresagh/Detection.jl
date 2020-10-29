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
		"--quote"
			help = "z coordinate of section"
			arg_type = Float64
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
	q = args["quote"]

	affine_matrix = Detection.Lar.t(0,0,q)

	Detection.detection_and_saves(output_folder, project_name, source, par, threshold, failed, N, k, affine_matrix	)
end

@time main()
