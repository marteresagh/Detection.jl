println("loading packages... ")

using ArgParse
using Detection
using Common
using AlphaStructures

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
		default = 5
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 20
	"--masterseeds","-s"
		help = "A text file with seeds list"
		arg_type = String
	end

	return parse_args(s)
end



# alpha shape
function save_alpha_shape_model(hyperplanes::Array{Hyperplane,1}, name_proj::String)
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		Detection.flushprintln("$i planes processed")

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		model = get_boundary_alpha_shape(hyperplane,plane)

		vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
		out = push!(out, Lar.Struct([(vertices, model[2])]))

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)
	FileManager.save_points_txt(name_proj*"_points.txt", V)
	FileManager.save_cells_txt(name_proj*"_edges.txt", EV)
	return V,EV
end

function get_boundary_alpha_shape(hyperplane::Hyperplane, plane::Plane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	threshold = Common.estimate_threshold(V,10)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
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

	hyperplanes, params, dirs = Detection.pc2plane(output_folder, project_name, PC, par, failed, N, k; masterseeds = masterseeds)

	for i in 1:length(hyperplanes)
		FileManager.save_hyperplane(joinpath(dirs.PLANE,"plane_$i.txt"), hyperplanes[i])
	end

	W,EW = save_alpha_shape_model(hyperplanes, joinpath(dirs.A_SHAPES,"a_shapes"))

end

@time main()
