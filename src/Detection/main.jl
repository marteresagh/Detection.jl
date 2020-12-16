"""
Main
"""
function pc2vectorize(
	folder::String,
	project_name::String,
	PC::PointCloud,
	par::Float64,
	failed::Int64,
	N::Int64,
	k::Int64,
	affine_matrix::Matrix,
	lines = true::Bool
	)


	flushprintln("=========== INIT =============")
	# output directory
	dirs = VectDirs(folder, project_name)

	if lines
		INPUT_PC = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	# 1. Initialization
	flushprintln("Search of possible outliers to remove: ")
	outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	flushprintln("$(length(outliers)) outliers")

	flushprintln()
	flushprintln("=========== PROCESSING =============")

	# threashold estimation
	threshold = Common.estimate_threshold(INPUT_PC,k)

	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

	# 2. Detection
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. Saves
	flushprintln()
	flushprintln("=========== SAVES =============")

	if lines
		saves_data(PC, params, hyperplanes, affine_matrix, dirs)
	end

	return hyperplanes,params
end


function saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, dirs::VectDirs)
	if !isempty(hyperplanes)
		io = open(joinpath(dirs.output_folder,"execution.prob"),"w")
		close(io)

		PC_fitted_2D = PointCloud(PC.coordinates[1:2,params.fitted],PC.rgbs[:,params.fitted])
		PC_fitted_3D = PointCloud(PC.coordinates[:,params.fitted],PC.rgbs[:,params.fitted])

		# lines
		# DXF/RAW
		flushprintln("Lines: saving...")
		flushprintln("Detect $(length(hyperplanes)) lines")
		FileManager.save_3D_lines_txt(joinpath(dirs.RAW,"segment3D.ext"), hyperplanes, affine_matrix)
		FileManager.save_2D_lines_txt(joinpath(dirs.RAW,"segment2D.ext"), hyperplanes)
		flushprintln("Lines: done...")

		# fitted
		flushprintln("Fitted points: saving...")
		flushprintln("Fitted $(length(params.visited)) points")
		# POINTCLOUDS/PARTITIONS
		FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"fitted.las"), PC_fitted_3D, "VECTORIZATION" )
		# DXF/RAW
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted2D.pnt"), PC_fitted_2D)
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted3D.pnt"), PC_fitted_3D)
		flushprintln("Fitted points: done...")

		# unfitted
		points_unfitted = setdiff(collect(1:PC.n_points),params.fitted)
		if !isempty(points_unfitted)
			flushprintln("Unfitted points: saving...")
			PC_unfitted_2D = PointCloud(PC.coordinates[1:2,points_unfitted],PC.rgbs[:,points_unfitted])
			PC_unfitted_3D = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
			flushprintln("Unfitted $(length(points_unfitted)) points")
			# POINTCLOUDS/PARTITIONS
			FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"unfitted.las"), PC_unfitted_3D, "VECTORIZATION" )
			# DXF/RAW
			FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted3D.pnt"), PC_unfitted_3D)
			FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted2D.pnt"), PC_unfitted_2D)
			flushprintln("Unfitted points: done...")
		end

		# POINTCLOUDS/FULL
		flushprintln("Slice: saving...")
		flushprintln(" $(PC.n_points) points in slice")
		FileManager.save_pointcloud(joinpath(dirs.FULL,"slice.las"), PC, "VECTORIZATION" )
		flushprintln("Slice: done...")
	end

end
