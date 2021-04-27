"""
	 save_partitions(PC::PointCloud, params::Initializer, affine_matrix::Matrix, dirs::Vect_1D_Dirs)
"""
function save_partitions(PC::PointCloud, params::Initializer, affine_matrix::Matrix, dirs::Vect_1D_Dirs)

	PC_fitted_2D = PointCloud(Common.apply_matrix(Common.inv(affine_matrix),PC.coordinates)[1:2,params.fitted],PC.rgbs[:,params.fitted])
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
		PC_unfitted_2D = PointCloud(Common.apply_matrix(Common.inv(affine_matrix),PC.coordinates)[1:2,points_unfitted],PC.rgbs[:,points_unfitted])
		PC_unfitted_3D = PointCloud(PC.coordinates[:,points_unfitted],PC.rgbs[:,points_unfitted])
		flushprintln("Unfitted $(length(points_unfitted)) points")
		# POINTCLOUDS/PARTITIONS
		FileManager.save_pointcloud(joinpath(dirs.PARTITIONS,"unfitted.las"), PC_unfitted_3D, "VECTORIZATION" )
		# DXF/RAW
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted3D.pnt"), PC_unfitted_3D)
		FileManager.save_points_rgbs_txt(joinpath(dirs.RAW,"unfitted2D.pnt"), PC_unfitted_2D)
	end

end

"""
	 write_line(s_2d::IOStream, s_3d::IOStream, line::Hyperplane, affine_matrix::Matrix)

Save line detected in 2D and 3D space.
"""
function write_line(s_2d::IOStream, s_3d::IOStream, line::Hyperplane, affine_matrix::Matrix)
	V = extrema_line(line)
	write(s_2d, "$(V[1,1]) $(V[2,1]) $(V[1,2]) $(V[2,2])\n")
	V1 = vcat(V,(zeros(size(V,2)))')
	V3D = Common.apply_matrix(affine_matrix,V1)
	write(s_3d, "$(V3D[1,1]) $(V3D[2,1]) $(V3D[3,1]) $(V3D[1,2]) $(V3D[2,2]) $(V3D[3,2])\n")
end


"""
	extrema_line(line::Hyperplane)::Points

"""
function extrema_line(line::Hyperplane)::Points
	max_value = -Inf
	min_value = +Inf
	points = line.inliers
	for i in 1:points.n_points
		p = points.coordinates[:,i] - line.centroid
		value = LinearAlgebra.dot(line.direction,p)
		if value > max_value
			max_value = value
		end
		if value < min_value
			min_value = value
		end
	end
	p_min = line.centroid + (min_value)*line.direction
	p_max = line.centroid + (max_value)*line.direction
	V = hcat(p_min,p_max)
	return V
end
