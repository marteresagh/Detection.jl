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
	end

	return parse_args(s)
end


function get_boundary_shapes(filename::String, hyperplanes::Array{Hyperplane,1})

	io = open(filename,"w")
	for i in 1:length(hyperplanes)

		hyperplane = hyperplanes[i]

		# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
		points = hyperplane.inliers.coordinates
		plane = Plane(hyperplane.direction..., Lar.dot(hyperplane.direction,hyperplane.centroid))
		T = Common.apply_matrix(Lar.inv(plane.matrix),points)[1:2,:]

		# 2. applica alpha shape con alpha = threshold
		filtration = AlphaStructures.alphaFilter(T);
		_, _, FV = AlphaStructures.alphaSimplex(T, filtration, threshold)

		# 3. estrai bordo
		EV_boundary = Common.get_boundary_edges(T,FV)

		# 4. salva i segmenti del bordo in 3D
		T = Common.points_projection_on_plane(points, hyperplane)
		for ev in EV_boundary
			write(io, "$(T[1,ev[1]]) $(T[2,ev[1]]) $(T[3,ev[1]]) $(T[1,ev[2]]) $(T[2,ev[2]]) $(T[3,ev[2]])\n")
		end

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end
	end

	close(io)
	return FileManager.load_segment(filename) # V,EV
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
	V,EV = get_boundary_shapes(filename::String, hyperplanes::Array{Hyperplanes,1})

	#TODO: da concludere process boundary edges
	#modello_finale = Detection.linearization(V,EV)
	#TODO: salvataggio

end

@time main()
