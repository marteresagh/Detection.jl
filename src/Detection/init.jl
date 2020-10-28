"""
programmino
"""
function detection_and_saves(PC::PointCloud,
	 						par::Float64,
							threshold::Float64,
							failed::Int64,
							N::Int64,
							k=10::Int64,
							affine_matrix::Matrix,
							filename::String,
							lines = true::Bool)

	# 1. ricerca degli outliers
	outliers = Common.outliers(PC, [1:PC.n_points...], k)

	# 2. faccio partire il processo
	if lines
		INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	no_seeds = copy(outliers)
	params = Initializer(INPUT_PC, par, threshold, failed, N, k, no_seeds)

	hyperplanes = Detection.iterate_random_detection(params)

	# 3. salvo tutto
	name = joinpath(folder,filename)
	saves_data(PC, params, hyperplanes, affine_matrix, name)

end


function saves_data(PC,params,hyperplanes,affine_matrix, path2name)
	points_fitted = setdiff([1:PC.n_points...],params.current_inds)
	PC_fitted = PointCloud(PC.coordinates[:,points_fitted],PC.rgbs[:,points_fitted])
	PC_unfitted = PointCloud(PC.coordinates[:,params.current_inds],PC.rgbs[:,params.current_inds])

	FileManager.save_lines_txt(path2name*"_lines.txt", hyperplanes, affine_matrix)

	FileManager.save_pointcloud(path2name*"_pts_fitted.las", PC_fitted, "DETECTION" )
	FileManager.save_points_rgbs_txt(path2name*"_pts_fitted.txt", PC_fitted)

	FileManager.save_pointcloud(path2name*"_pts_unfitted.las", PC_unfitted, "DETECTION")
	FileManager.save_points_rgbs_txt(path2name*"_pts_unfitted.txt", PC_unfitted)
end
