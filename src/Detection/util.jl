"""
First seed.
"""
# init_seed = findall(x->x == given_seed, params.current_inds)[1] # se dato
function seedpoint(points::Lar.Points, params::Initializer; given_seed = rand(1:size(points,2))::Int64)

	kdtree = Common.KDTree(points)
	idxseeds = Common.neighborhood(kdtree,points,[given_seed],Int64[],params.threshold,params.k)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = Common.minresidual(seeds,hyperplane)
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

	res = Common.residual(hyperplane).([c[:] for c in eachcol(pc_on_hyperplane.coordinates)])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	@assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"  #0.005 che valore è?? come generalizzare??

end
