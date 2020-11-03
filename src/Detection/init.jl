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

	if lines
		INPUT_PC = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,:], PC.rgbs)
	else
		INPUT_PC = PC
	end

	# 1. ricerca degli outliers
	flushprintln("Search of possible outliers to remove: ")
	outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	flushprintln("$(length(outliers)) outliers")

	flushprintln()
	flushprintln("=========== PROCESSING =============")
	# 2. faccio partire il processo
	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. salvo tutto
	flushprintln()
	flushprintln("=========== SAVES =============")
	name = joinpath(proj_folder,project_name)

	saves_data(PC, params, hyperplanes, affine_matrix, name)

	return hyperplanes,params
end


function saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, path2name::String)
	PC_fitted = PointCloud(PC.coordinates[:,params.fitted],PC.rgbs[:,params.fitted])

	flushprintln("Lines: saving...")
	flushprintln("Detect $(length(hyperplanes)) lines")
	FileManager.save_lines_txt(path2name*"_vectorized_1D.txt", hyperplanes, affine_matrix)
	flushprintln("Lines: done...")

	flushprintln("Fitted points: saving...")
	flushprintln("Fitted $(length(params.visited)) points")
	FileManager.save_pointcloud(path2name*"_fitted_points.las", PC_fitted, "DETECTION" )
	FileManager.save_points_rgbs_txt(path2name*"_fitted_points.txt", PC_fitted)
	flushprintln("Fitted points: done...")

	points_unfitted = setdiff(collect(1:PC.n_points),params.fitted)
	if !isempty(points_unfitted)
		PC_unfitted = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
		flushprintln("Unfitted points: saving...")
		flushprintln("Unfitted $(length(points_unfitted)) points")
		FileManager.save_pointcloud(path2name*"_unfitted_points.las", PC_unfitted, "DETECTION")
		FileManager.save_points_rgbs_txt(path2name*"_unfitted_points.txt", PC_unfitted)
		flushprintln("Unfitted points: done...")
	end

	if !isempty(params.outliers)
		red_color =  hcat(fill(LasIO.N0f16.([1,.0,.0]),length(params.outliers))...)
		PC_outliers = PointCloud(PC.coordinates[:,params.outliers],red_color)
		flushprintln("Outliers points: saving...")
		flushprintln("Marked $(length(params.outliers)) outliers")
		FileManager.save_pointcloud(path2name*"_outliers_points.las", PC_outliers, "DETECTION")
		FileManager.save_points_rgbs_txt(path2name*"_outliers_points.txt", PC_outliers)
		flushprintln("Outliers points: done...")
	end
end
