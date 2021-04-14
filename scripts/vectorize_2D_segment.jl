println("loading packages... ")

using ArgParse
using Common
using OrthographicProjection

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



function save_full_inliers(potree::String, folders::Array{String,1}, hyperplanes::Array{Hyperplane,1}, thickness::Float64)
	n_planes = length(folders)
	Threads.@threads for i in 1:n_planes
		Common.flushprintln()
		Common.flushprintln("==========================================================")
		Common.flushprintln("=================== $i of $n_planes ======================")
		Common.flushprintln("==========================================================")

		# segmentation: to extract all inliers
		Common.flushprintln()
		Common.flushprintln("Segmentation....")
		Common.flushprintln("-----------------------------------------------------------")
		inliers_points = hyperplanes[i].inliers.coordinates
		aabb = Common.boundingbox(inliers_points)
		plane = Plane(hyperplanes[i].direction,hyperplanes[i].centroid)
		model = Common.getmodel(plane, thickness, aabb)
		OrthographicProjection.segment(potree, joinpath(folders[i],"full_inliers.las"), model)
		Common.flushprintln("-----------------------------------------------------------")
		Common.flushprintln("Segmentation.... Done")
	end
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	thickness = args["thickness"]

	Common.flushprintln("== Parameters ==")
	Common.flushprintln("Source  =>  $source")
	Common.flushprintln("Output folder  =>  $output_folder")
	Common.flushprintln("Project name  =>  $project_name")
	Common.flushprintln("Thickness  =>  $thickness")

	folders = get_plane_folders(output_folder, project_name)
	hyperplanes, OBBs = get_hyperplanes(folders)

	save_full_inliers(source, folders, hyperplanes, thickness)

end

@time main()
