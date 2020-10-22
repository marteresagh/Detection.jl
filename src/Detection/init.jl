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

	idxs, dists = knn(kdtree, points[:,randindex], k, false)
	filter = [dist<=threshold for dist in dists]
	idxseeds = idxs[filter]

	seeds = points[:,idxseeds]

	direction, centroid = Common.LinearFit(seeds)
	#dim = size(points,1)
	# if dim = 3
	# 	direction,centroid = Common.Plane_fit(seeds)
	# elseif dim = 2
	# 	direction,centroid = Common.Line_Fit(seeds)
	# end

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane, randindex
end
