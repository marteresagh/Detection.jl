"""
programmino
"""
function detection_and_saves(
							folder::String,
							project_name::String,
							source::String,
	 						par::Float64,
							threshold::Float64,
							failed::Int64,
							N::Int64,
							k::Int64,
							affine_matrix::Matrix,
							lines = true::Bool
							)


	flushprintln("=========== INIT =============")

	@assert isdir(folder) "$folder not an existing folder"
	proj_folder = joinpath(folder,project_name)

	if !isdir(proj_folder)
		mkdir(proj_folder)
	end

	PC = FileManager.las2pointcloud(source)

	flushprintln(" Found possible outliers to remove ")
	# 1. ricerca degli outliers
	outliers = Common.outliers(PC, [1:PC.n_points...], k)

	if lines
		INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	flushprintln("=========== PROCESSING =============")
	# 2. faccio partire il processo
	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. salvo tutto
	flushprintln("=========== SAVES =============")
	name = joinpath(proj_folder,project_name)
	saves_data(PC, params, hyperplanes, affine_matrix, name)

	return hyperplanes,params
end


function saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, path2name::String)
	points_unfitted = setdiff([1:PC.n_points...],params.visited)
	PC_fitted = PointCloud(PC.coordinates[:,params.visited],PC.rgbs[:,params.visited])
	PC_unfitted = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
	red_color =  hcat(fill([1,.0,.0],length(params.outliers))...)
	PC_outliers = PointCloud(PC.coordinates[:,params.outliers],red_color)

	flushprintln("Lines: saving...")
	FileManager.save_lines_txt(path2name*"_lines.txt", hyperplanes, affine_matrix)
	flushprintln("Lines: done...")

	flushprintln("Fitted points: saving...")
	FileManager.save_pointcloud(path2name*"_pts_fitted.las", PC_fitted, "DETECTION" )
	FileManager.save_points_rgbs_txt(path2name*"_pts_fitted.txt", PC_fitted)
	flushprintln("Fitted points: done...")

	flushprintln("Unfitted points: saving...")
	FileManager.save_pointcloud(path2name*"_pts_unfitted.las", PC_unfitted, "DETECTION")
	FileManager.save_points_rgbs_txt(path2name*"_pts_unfitted.txt", PC_unfitted)
	flushprintln("Unfitted points: done...")

	flushprintln("Outliers points: saving...")
	FileManager.save_pointcloud(path2name*"_pts_outliers.las", PC_outliers, "DETECTION")
	FileManager.save_points_rgbs_txt(path2name*"_pts_outliers.txt", PC_outliers)
	flushprintln("Outliers points: done...")
end
