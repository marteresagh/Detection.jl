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
	visited::Array{Int64,1}
	current_inds::Array{Int64,1}

	Initializer(PC, par, threshold, failed, N, visited, current_inds) = new(PC, par, threshold, failed, N, visited, current_inds)
	Initializer(PC, par, threshold, failed, N, visited) = new(PC, par, threshold, failed, N, visited, [1:PC.n_points...])
	Initializer(PC, par, threshold, failed, N) = new(PC, par, threshold, failed, N, [], [1:PC.n_points...])
end
