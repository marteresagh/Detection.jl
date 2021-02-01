"""
	 save_partitions(PC::PointCloud, params::Initializer, affine_matrix::Matrix, dirs::Vect_1D_Dirs)
"""
function save_partitions(PC::PointCloud, params::Initializer, affine_matrix::Matrix, dirs::Vect_1D_Dirs)

	PC_fitted_2D = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,params.fitted],PC.rgbs[:,params.fitted])
	PC_fitted_3D = PointCloud(PC.coordinates[:,params.fitted],PC.rgbs[:,params.fitted])

	# fitted
	flushprintln("Fitted $(length(params.fitted)) points")
	# POINTCLOUDS/PARTITIONS
	FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"fitted.las"), PC_fitted_3D, "VECTORIZATION" )
	# DXF/RAW
	FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted2D.pnt"), PC_fitted_2D)
	FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted3D.pnt"), PC_fitted_3D)

	# unfitted
	points_unfitted = setdiff(collect(1:PC.n_points),params.fitted)
	if !isempty(points_unfitted)
		PC_unfitted_2D = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,points_unfitted],PC.rgbs[:,points_unfitted])
		PC_unfitted_3D = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
		flushprintln("Unfitted $(length(points_unfitted)) points")
		# POINTCLOUDS/PARTITIONS
		FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"unfitted.las"), PC_unfitted_3D, "VECTORIZATION" )
		# DXF/RAW
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted3D.pnt"), PC_unfitted_3D)
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted2D.pnt"), PC_unfitted_2D)
	end

end
