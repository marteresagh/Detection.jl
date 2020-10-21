"""
Find seed point randomly.
"""
function seedpoint(points::Lar.Points, threshold::Float64, k=10::Int64)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		return findmin([residual(points[:,i],hyperplane) for i in 1:size(points,2)])[2]#TODO residual
	end

	kdtree = KDTree(points)
	randindex = rand(1:size(points,2))

	idxs, dists = knn(kdtree, points[:,randindex], k, false)
	filter = [dist<=threshold for dist in dists]
	idxseeds = idxs[filter]

	seeds = points[:,idxseeds]

	dim = size(points,1)
	if dim = 3
		direction,centroid = Plane_fit(seeds)
	elseif dim = 2
		direction,centroid = Line_Fit(seeds)
	end

	hyperplane = Hyperplane(direction,centroid)
	minresidual = minresidual(seeds,hyperplane)
	seed = idxseeds[minresidual]

	return seed, hyperplane
end
