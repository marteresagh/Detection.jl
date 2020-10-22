"""
Find seed point randomly.
"""
function seedpoint(PC::PointCloud, currents_inds::Array{Int64,1}, threshold::Float64, k=10::Int64)

	points = PC.coordinates[:,currents_inds]

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = Common.KDTree(points)
	randindex = rand(1:size(points,2))

	idxseeds = Common.neighborhood(kdtree,PointCloud(points),[randindex],Int64[],threshold)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return currents_inds[seed], hyperplane, randindex
end
