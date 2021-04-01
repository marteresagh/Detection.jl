println("loading packages... ")

using ArgParse
using Detection
using Common
using OrthographicProjection
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
	"--thickness"
		help = "Thickness"
		arg_type = Float64
	end

	return parse_args(s)
end



function save_full_inliers(potree::String, folders::Array{String,1}, hyperplanes::Array{Hyperplane,1}, thickness::Float64)
	n_planes = length(folders)
	Threads.@threads for i in 1:n_planes
		Detection.flushprintln()
		Detection.flushprintln("==========================================================")
		Detection.flushprintln("=================== $i of $n_planes ======================")
		Detection.flushprintln("==========================================================")

		# segmentation: to extract all inliers
		Detection.flushprintln()
		Detection.flushprintln("Segmentation....")
		Detection.flushprintln("-----------------------------------------------------------")
		inliers_points = hyperplanes[i].inliers.coordinates
		aabb = Common.boundingbox(inliers_points)
		plane = Plane(hyperplanes[i].direction,hyperplanes[i].centroid)
		model = Common.getmodel(plane, thickness, aabb)
		OrthographicProjection.segment(potree, joinpath(folders[i],"full_inliers.las"), model)
		Detection.flushprintln("-----------------------------------------------------------")
		Detection.flushprintln("Segmentation.... Done")
	end
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	thickness = args["thickness"]

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Thickness  =>  $thickness")

	folders = FileManager.get_plane_folders(output_folder, project_name)
	hyperplanes, OBBs = FileManager.get_hyperplanes(folders)

	save_full_inliers(source, folders, hyperplanes, thickness)

end

@time main()
