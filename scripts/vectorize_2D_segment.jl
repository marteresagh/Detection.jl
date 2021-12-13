println("loading packages... ")

using ArgParse
using Common
using Clipping
using Detection

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
	"--thickness"
		help = "Thickness"
		arg_type = Float64
	end

	return parse_args(s)
end


function save_full_inliers(potree::String, folders::Array{String,1}, thickness::Float64)
	n_planes = length(folders)
	Threads.@threads for i in 1:n_planes
		println()
		println("==========================================================")
		println("=================== $i of $n_planes ======================")
		println("==========================================================")

		io = open(joinpath(folders[i],"finite_plane.txt"), "r")
		lines = readlines(io)
		close(io)

		b = [tryparse.(Float64,split(lines[i], " ")) for i in 1:length(lines)]
		normal = [b[1][1],b[1][2],b[1][3]]
		centroid = normal*b[1][4]
		inliers = FileManager.load_points(joinpath(folders[i],"inliers.txt"))

		hyperplane = Detection.Hyperplane(PointCloud(inliers[1:3,:],inliers[4:6,:]), normal, centroid)

		# segmentation: to extract all inliers
		println()
		println("Segmentation....")
		println("-----------------------------------------------------------")
		inliers_points = hyperplane.inliers.coordinates
		aabb = Common.boundingbox(inliers_points)
		plane = Plane(hyperplane.direction,hyperplane.centroid)
		model = Common.getmodel(plane, thickness, aabb)
		Clipping.clip(potree, joinpath(folders[i],"full_inliers.las"), model, nothing)
		println("-----------------------------------------------------------")
		println("Segmentation.... Done")
		FileManager.successful(true, folders[i]; filename = "vectorize_2D_segment.probe")
	end
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	thickness = args["thickness"]

	println("== Parameters ==")
	println("Source  =>  $source")
	println("Output folder  =>  $output_folder")
	println("Project name  =>  $project_name")
	println("Thickness  =>  $thickness")

	folders = Detection.get_plane_folders(output_folder, project_name)
#	hyperplanes, OBBs = Detection.get_hyperplanes(folders)

	save_full_inliers(source, folders, thickness)
end

@time main()
