println("loading packages... ")

using ArgParse
using Detection
using Common

println("packages OK")

# function boundary_shape

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"--output", "-o"
		help = "Output folder"
		required = true
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	output_folder = args["output"]

	println("== Parameters ==")
	println("Output folder  =>  $output_folder")


	flush(stdout)

	# read output CGAL
	points, total_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
	points, candidate_faces = Detection.read_OFF(joinpath(output_folder, "candidate_faces.off"))
	# remove faces
	# points,faces = remove_faces(points,total_faces, candidate_faces) # TODO
	# clustering candidate faces
	edges, triangles, regions = Detection.clustering_faces(points, faces)
	# get polygons
	polygons = Detection.get_polygons(points, triangles, clusters)
	#save boundary polygons
	Detection.save_boundary_polygons(output_folder, points, polygons)

end

@time main()
