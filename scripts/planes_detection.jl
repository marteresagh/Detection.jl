println("loading packages...")

using ArgParse
using Detection
using FileManager
using Search

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
		default = 40
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 30
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
	masterseeds = args["masterseeds"]

	println()
	println("== Parameters ==")
	println("Source  =>  $source")
	println("Output folder  =>  $output_folder")
	println("Project name  =>  $project_name")
	println("Parameter  =>  $par")
	println("Seeds =>  $(args["masterseeds"])")
	println("N. of failed  =>  $failed")
	println("N. of inliers  =>  $N")
	println("N. of k-nn  =>  $k")
	println("Level of detail  =>  $lod")

	flush(stdout)
	println()
	println("=========== INIT =============")
	# input point cloud
	PC = FileManager.source2pc(source, lod)
	# output directory
	project_folder = FileManager.mkdir_project(output_folder,project_name)
	tmp_folder = FileManager.mkdir_project(project_folder,"tmp")
	planes_folder = FileManager.mkdir_project(tmp_folder,"PLANES")
	faces_folder = FileManager.mkdir_project(tmp_folder,"FACES")
	pc_folder = FileManager.mkdir_project(project_folder,"POINTCLOUDS")
	cgal_folder = FileManager.mkdir_project(project_folder,"SEGMENTS")
	polygons_folder = FileManager.mkdir_project(project_folder,"POLYGONS")


	println("Pointcloud: $(PC.n_points) points")

	# seeds
	seeds = Int64[]
	if !isnothing(masterseeds) # if seeds are provided
		println("Read seeds from file")
		given_seeds = FileManager.load_points(masterseeds)
		seeds = Search.consistent_seeds(PC).([c[:] for c in eachcol(given_seeds)])
	end

	params = Detection.Initializer(PC, par, failed,	N, k)

	# 2. Detection
	println()
	println("=========== PROCESSING =============")
	i = Detection.iterate_planes_detection(params, planes_folder; seeds = seeds)

	# 3. Saves
	println()
	println("=========== RESULTS =============")
	# saving planes
	println("$i planes detected")


	if i!=0
		print("Saving plane segments in .ply... ")
		plane_dirs = Detection.get_plane_folders(tmp_folder,"PLANES")
		hyperplanes, _ = Detection.get_hyperplanes(plane_dirs)
		Detection.refine_planes!(hyperplanes)
		Detection.save_plane_segments_in_ply(hyperplanes, joinpath(cgal_folder,"segments.ply"))
		println("Done: $(length(hyperplanes)) planes.")

		# Saving points
		print("Saving: Fitted and unfitted points... ")
		point_cloud = params.PC

		fitted_idx = params.fitted
		if !isempty(fitted_idx)
			PC_fitted = Detection.PointCloud(point_cloud.coordinates[:,fitted_idx],point_cloud.rgbs[:,fitted_idx])
			FileManager.save_points_rgbs_txt(joinpath(pc_folder,"fitted_points.txt"), PC_fitted)
			FileManager.save_pointcloud(joinpath(pc_folder,"fitted_points.las"), PC_fitted, "PLANES DETECTION")
		end

		unfitted_idx = setdiff(collect(1:point_cloud.n_points),fitted_idx)
		if !isempty(unfitted_idx)
			PC_unfitted = Detection.PointCloud(point_cloud.coordinates[:,unfitted_idx],point_cloud.rgbs[:,unfitted_idx])
			FileManager.save_points_rgbs_txt(joinpath(pc_folder,"unfitted_points.txt"), PC_unfitted)
			FileManager.save_pointcloud(joinpath(pc_folder,"unfitted_points.las"), PC_unfitted, "PLANES DETECTION")
		end

		println("Done.")
	else
		println("No planes found")
	end



	FileManager.successful(i!=0, project_folder; filename = "plane_detection.probe")


end

@time main()
