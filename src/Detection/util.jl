"""
Find first seed randomly.
"""
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
	# TODO forse è buono
	# res = Common.residual(hyperplane).([pc_on_hyperplane.coordinates[:,i] for i in 1:pc_on_hyperplane.n_points])
	# mu = Statistics.mean(res)
	# rho = Statistics.std(res)
	# @assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"
end

# function validity(hyperplane::Hyperplane, params::Initializer, cluster, all_visited)
# 	# VALIDITY
# 	pc_on_hyperplane = hyperplane.inliers
# 	#@show length(cluster)/length(all_visited)
# 	#@assert  pc_on_hyperplane.n_points > params.N "not valid" #TODO da ottimizzare
#
# 	E,_ = Common.DrawLine(hyperplane, 0.0)
# 	dist = Lar.norm(E[:,1]-E[:,2])
# 	rho = pc_on_hyperplane.n_points/dist
# 	@assert  rho > N "not valid"  #da automatizzare
# end
