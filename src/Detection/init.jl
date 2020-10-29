"""
programmino
"""
function detection_and_saves(
							folder::String,
							filename::String,
							PC::PointCloud,
	 						par::Float64,
							threshold::Float64,
							failed::Int64,
							N::Int64,
							k::Int64,
							affine_matrix::Matrix,
							lines = true::Bool
							)

	# 1. ricerca degli outliers
	outliers = Common.outliers(PC, [1:PC.n_points...], k)

	# 2. faccio partire il processo
	if lines
		INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. salvo tutto
	name = joinpath(folder,filename)
	saves_data(PC, params, hyperplanes, affine_matrix, name)

	return hyperplanes,params
end


function saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, path2name::String)
	points_unfitted = setdiff([1:PC.n_points...],params.visited)
	PC_fitted = PointCloud(PC.coordinates[:,params.visited],PC.rgbs[:,params.visited])
	PC_unfitted = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
	PC_outliers = PointCloud(PC.coordinates[:,params.outliers],PC.rgbs[:,params.outliers])

	FileManager.save_lines_txt(path2name*"_lines.txt", hyperplanes, affine_matrix)

	FileManager.save_pointcloud(path2name*"_pts_fitted.las", PC_fitted, "DETECTION" )
	FileManager.save_points_rgbs_txt(path2name*"_pts_fitted.txt", PC_fitted)

	FileManager.save_pointcloud(path2name*"_pts_unfitted.las", PC_unfitted, "DETECTION")
	FileManager.save_points_rgbs_txt(path2name*"_pts_unfitted.txt", PC_unfitted)

	FileManager.save_pointcloud(path2name*"_pts_outliers.las", PC_outliers, "DETECTION")
	FileManager.save_points_rgbs_txt(path2name*"_pts_outliers.txt", PC_outliers)
end
