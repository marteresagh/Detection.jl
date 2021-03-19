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
	"--par"
		help = "Parameter"
		arg_type = Float64
		default = 0.01
	"--angle"
		help = "Angle"
		arg_type = Float64
		default = pi/8
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 30
	end

	return parse_args(s)
end



function save_boundary(potree::String, folders::Array{String,1}, hyperplanes::Array{Hyperplane,1}, thickness::Float64, par::Float64, angle::Float64, k::Int64)
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

		# alpha shape of full inliers
		Detection.flushprintln()
		Detection.flushprint("Alpha shapes....")
		file = joinpath(folders[i],"full_inliers.las")
		#######################################
		# se troppi punti si possono decimare #
		#######################################
		PC = FileManager.las2pointcloud(file)
		points = PC.coordinates
		plane = Plane(points)
		V = Common.apply_matrix(plane.matrix,points)[1:2,:]

		DT = Common.delaunay_triangulation(V)
		filtration = AlphaStructures.alphaFilter(V,DT);
		threshold = Common.estimate_threshold(V,k)
		_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
		Detection.flushprintln("Done")

		# boundary extraction
		Detection.flushprintln()
		Detection.flushprint("Boundary extraction....")
		EV_boundary = Common.get_boundary_edges(V,FV)
		w,EW = Lar.simplifyCells(V,EV_boundary)
		W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))
		model = (W,EW)
		Detection.flushprintln("Done")

		# boundary semplification
		Detection.flushprint("Boundary semplification....")
		V, EV = Detection.simplify_model(model; par = par, angle = angle)
		Detection.flushprintln("Done")

		# save data
		Detection.flushprintln()
		Detection.flushprint("Saves....")
		V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
		FileManager.save_points_txt(joinpath(folders[i],"boundary_points2D.txt"), V2D)
		FileManager.save_points_txt(joinpath(folders[i],"boundary_points3D.txt"), V)
		FileManager.save_connected_components(joinpath(folders[i],"boundary_edges.txt"), V, EV)
		Detection.flushprintln("Done")
		Detection.flushprintln()

	end
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	thickness = args["thickness"]
	par = args["par"]
	angle = args["angle"]
	k = args["k"]

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Thickness  =>  $thickness")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("Angle  =>  $angle")

	folders = FileManager.get_plane_folders(output_folder, project_name)
	hyperplanes, OBBs = FileManager.get_hyperplanes(folders)

	save_boundary(source, folders, hyperplanes, thickness, par, angle, k)

end

@time main()
