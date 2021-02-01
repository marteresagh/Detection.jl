# """
# 	saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1},affine_matrix::Matrix, dirs::VectDirs)
# """
# function saves_data(PC::PointCloud,params::Initializer,hyperplanes::Array{Hyperplane,1}, affine_matrix::Matrix, dirs::VectDirs)
# 	FileManager.successful(!isempty(hyperplanes), dirs.output_folder)
# 	if !isempty(hyperplanes)
#
# 		PC_fitted_2D = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,params.fitted],PC.rgbs[:,params.fitted])
# 		PC_fitted_3D = PointCloud(PC.coordinates[:,params.fitted],PC.rgbs[:,params.fitted])
#
# 		# lines
# 		# DXF/RAW
# 		flushprintln("Lines: saving...")
# 		flushprintln("Detect $(length(hyperplanes)) lines")
# 		FileManager.save_3D_lines_txt(joinpath(dirs.RAW,"segment3D.ext"), hyperplanes, affine_matrix)
# 		FileManager.save_2D_lines_txt(joinpath(dirs.RAW,"segment2D.ext"), hyperplanes)
# 		flushprintln("Lines: done...")
#
# 		# fitted
# 		flushprintln("Fitted points: saving...")
# 		flushprintln("Fitted $(length(params.fitted)) points")
# 		# POINTCLOUDS/PARTITIONS
# 		FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"fitted.las"), PC_fitted_3D, "VECTORIZATION" )
# 		# DXF/RAW
# 		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted2D.pnt"), PC_fitted_2D)
# 		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"fitted3D.pnt"), PC_fitted_3D)
# 		flushprintln("Fitted points: done...")
#
# 		# unfitted
# 		points_unfitted = setdiff(collect(1:PC.n_points),params.fitted)
# 		if !isempty(points_unfitted)
# 			flushprintln("Unfitted points: saving...")
# 			PC_unfitted_2D = PointCloud(Common.apply_matrix(Lar.inv(affine_matrix),PC.coordinates)[1:2,points_unfitted],PC.rgbs[:,points_unfitted])
# 			PC_unfitted_3D = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
# 			flushprintln("Unfitted $(length(points_unfitted)) points")
# 			# POINTCLOUDS/PARTITIONS
# 			FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"unfitted.las"), PC_unfitted_3D, "VECTORIZATION" )
# 			# DXF/RAW
# 			FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted3D.pnt"), PC_unfitted_3D)
# 			FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted2D.pnt"), PC_unfitted_2D)
# 			flushprintln("Unfitted points: done...")
# 		end
#
# 		# POINTCLOUDS/FULL
# 		flushprintln("Slice: saving...")
# 		flushprintln(" $(PC.n_points) points in slice")
# 		FileManager.save_pointcloud(joinpath(dirs.FULL,"slice.las"), PC, "VECTORIZATION" )
# 		flushprintln("Slice: done...")
# 	end
#
# end
#
