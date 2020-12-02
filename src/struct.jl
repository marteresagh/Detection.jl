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

	Initializer(PC, par, threshold, failed, N, k, outliers, current_inds) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), current_inds,Int64[])
	Initializer(PC, par, threshold, failed, N, k, outliers) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), collect(1:PC.n_points),Int64[])
	Initializer(PC, par, threshold, failed, N, k) = new(PC, par, threshold, failed, N, k, Int64[], Int64[], collect(1:PC.n_points),Int64[])
end
