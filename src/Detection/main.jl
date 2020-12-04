"""
generate input point cloud
"""
function source2pc(source::String, lod::Union{Nothing,Int64})

	if isdir(source) # se source è un potree
		Detection.flushprintln("Potree struct")
		cloud_metadata = Detection.CloudMetadata(source)

		if lod == -1
			trie = potree2trie(source)
			max_level = FileManager.max_depth(trie)
			all_files = FileManager.get_all_values(trie)
			PC = FileManager.las2pointcloud(all_files...)
			return PC
		else
			all_files = FileManager.get_files_in_potree_folder(source,lod)
			PC = FileManager.las2pointcloud(all_files...)
			return PC
		end

	elseif isfile(source) # se source è un file
		PC = FileManager.las2pointcloud(source)
		return PC
	end

end

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
	proj_folder = FileManager.mkdir_project(folder,project_name)

	proj_folder3D = FileManager.mkdir_project(proj_folder,"3D")

	if lines
		proj_folder2D = FileManager.mkdir_project(proj_folder,"2D")
	end

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
	threshold = estimate_threshold(INPUT_PC,k)

	# 2. Detection
	params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)
	hyperplanes = Detection.iterate_random_detection(params)

	# 3. Saves
	flushprintln()
	flushprintln("=========== SAVES =============")

	if lines
		name3D = joinpath(proj_folder3D,project_name)
		saves_3D_data(PC, params, hyperplanes, affine_matrix, name3D)
		name2D = joinpath(proj_folder2D,project_name)
		saves_2D_data(INPUT_PC, params, hyperplanes, name2D)
	end

	return hyperplanes,params
end


function saves_3D_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, path2name::String)
	PC_fitted = PointCloud(PC.coordinates[:,params.fitted],PC.rgbs[:,params.fitted])

	flushprintln("Lines: saving...")
	flushprintln("Detect $(length(hyperplanes)) lines")
	FileManager.save_3D_lines_txt(path2name*"_vectorized_1D.txt", hyperplanes, affine_matrix)
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


function saves_2D_data(PC::PointCloud, params::Initializer, hyperplanes::Array{Hyperplane,1}, path2name::String)
	PC_fitted = PointCloud(PC.coordinates[:,params.fitted], PC.rgbs[:,params.fitted])

	flushprintln("Lines: saving...")
	flushprintln("Detect $(length(hyperplanes)) lines")
	FileManager.save_2D_lines_txt(path2name*"_vectorized_1D.txt", hyperplanes)
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
