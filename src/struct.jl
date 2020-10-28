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

mutable struct initParams
	PC::PointCloud
	par::Float64
	threshold::Float64
	failed::Int64
	N::Int64
	visited::Array{Int64,1}
	current_inds::Array{Int64,1}
	# punti_random_iniziali::Array{Array{Float64,1},1}
	# possible_seeds::Array{Lar.Points,1}
end
