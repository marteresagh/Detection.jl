"""
Find seed point randomly.
"""
function seedpoint(points::Lar.Points, params::initParams, k=10::Int64)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = Common.KDTree(points)
	randindex = rand(1:size(points,2))

	push!(params.punti_random_iniziali,points[:,randindex])

	
	idxseeds = Common.neighborhood(kdtree,points,[randindex],Int64[],params.threshold)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane
end
