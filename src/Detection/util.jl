"""
Find first seed randomly.
"""
function new_seedpoint(points::Lar.Points, params::Initializer)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = Common.KDTree(points)
	randindex = rand(1:size(points,2))
	idxseeds = NearestNeighbors.inrange(kdtree, points[:,randindex], 3*params.threshold)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	#TODO forse va usato come marcatore di punti da non considerare come seed
	@assert  max(Common.residual(hyperplane).([seeds[:,c] for c in 1:size(seeds,2)])...) < params.par "no seed"
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane
end


function seedpoint(points::Lar.Points, params::Initializer)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = Common.KDTree(points)
	randindex = rand(1:size(points,2))

	idxseeds = Common.neighborhood(kdtree,points,[randindex],Int64[],params.threshold,params.k)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane
end

"""
remove from inds the i-th element described in todel
"""
function remove_points!(inds::Array{Int64,1},todel::Array{Int64,1})
	setdiff!(inds,todel)
end

"""
Assert the detected hyperplane is valid / interesting
"""
function validity(hyperplane::Hyperplane, params::Initializer)
	# VALIDITY
	pc_on_hyperplane = hyperplane.inliers
	@assert  pc_on_hyperplane.n_points > params.N "not valid"

	res = Common.residual(hyperplane).([pc_on_hyperplane.coordinates[:,i] for i in 1:pc_on_hyperplane.n_points])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	@assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"  #0.005 che valore è?? come generalizzare??

end
