println("loading packages... ")

using ArgParse
using Detection

println("packages OK")

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Potree directory"
		required = true
	"--projectname", "-p"
		help = "Project name"
		required = true
	"--output", "-o"
		help = "Output folder"
		required = true
	"--par"
		help = "Distance to line"
		arg_type = Float64
		required = true
	"--lod"
		help = "Level of detail. If -1, all points are taken"
		arg_type = Int64
		default = -1
	"--failed"
		help = "Number of failed before exit"
		arg_type = Int64
		default = 100
	"--validity"
		help = "Minimum number of inliers"
		arg_type = Int64
		default = 5
	"--k"
		help = "Number of neighbors"
		arg_type = Int64
		default = 30
	"--plane"
		help = "a, b, c, d parameters described the plane"
		arg_type = String
		required = true
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
	plane = args["plane"]
	masterseeds = args["masterseeds"]

	# plane description
	b = tryparse.(Float64,split(plane, " "))
	@assert length(b) == 4 "$plane: Please described the plane in Hessian normal form"
	plane = Detection.Plane(b[1],b[2],b[3],b[4])

	PC = Detection.FileManager.source2pc(source, lod)
	# affine_matrix = Detection.Lar.approxVal(16).(plane.matrix) # rotation matrix

	points = PC.coordinates
	points2D = Detection.Common.apply_matrix(plane.matrix,points)[1:2,:]
	R = Detection.Common.basis_minimum_OBB_2D(points2D)
	affine_matrix = Detection.Lar.approxVal(16).(Detection.Common.matrix4(Detection.Lar.inv(R))*plane.matrix) # rotation matrix


	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("Seeds =>  $(args["masterseeds"])")
	Detection.flushprintln("N. of failed  =>  $failed")
	Detection.flushprintln("N. of inliers  =>  $N")
	Detection.flushprintln("N. of k-nn  =>  $k")
	Detection.flushprintln("Affine matrix =>  $affine_matrix")


	Detection.pc2lines(output_folder, project_name, PC, par, failed, N, k, affine_matrix; masterseeds = masterseeds)
end

@time main()
