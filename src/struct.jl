# struct Parameters
# 	pointcloud::PointCloud
# 	cloudMetadata::CloudMetadata
# 	LOD::UInt16
# 	par::Float64
# 	output::String
# 	rnd::Bool
# 	seedPoints::Lar.Points
# 	random::Bool
# 	failed::UInt16
# end

mutable struct Initializer
	PC::PointCloud
	par::Float64
	threshold::Float64
	failed::Int64
	N::Int64
	k::Int64
	outliers::Array{Int64,1}
	visited::Array{Int64,1}
	current_inds::Array{Int64,1}
	fitted::Array{Int64,1}

	Initializer(PC, par, threshold, failed, N, k, outliers, current_inds) = new(PC, par, threshold, failed, N, k, outliers, visited, current_inds,[])
	Initializer(PC, par, threshold, failed, N, k, outliers) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), [1:PC.n_points...],[])
	Initializer(PC, par, threshold, failed, N, k) = new(PC, par, threshold, failed, N, k, [], [], [1:PC.n_points...],[])
end
