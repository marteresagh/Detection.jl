"""
Find seed point randomly.
"""
function seedpoint(points::Lar.Points, threshold::Float64, k=10::Int64)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = KDTree(points)
	randindex = rand(1:size(points,2))

#TODO migliorare questo
	idxs, dists = knn(kdtree, points[:,randindex], k, false)
	filter = [dist<=threshold for dist in dists]
	idxseeds = idxs[filter]

	seeds = points[:,idxseeds]

	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane, randindex
end
