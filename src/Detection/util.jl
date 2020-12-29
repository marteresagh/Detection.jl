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

	res = Common.residual(hyperplane).([pc_on_hyperplane.coordinates[:,i] for i in 1:pc_on_hyperplane.n_points])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	@assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"  #0.005 che valore è?? come generalizzare??

end

"""
Corners detection
"""
function corners_detection(INPUT_PC::PointCloud, threshold::Float64, k=30::Int64, current_inds = collect(1:INPUT_PC.n_points)::Array{Int64,1}) # TODO usare KNN o i vicini in un certo range?
	points = INPUT_PC.coordinates[:, current_inds]
	npoints = size(points,2)
	corners = fill(false,npoints)
	curvs = fill(0.,npoints)
#	balltree = Common.BallTree(points)
	kdtree = Common.KDTree(points)
	for i in 1:npoints
		# TODO verificare che i vicini ci siano e che il valore della curvatura non sia NaN
		#N = Common.inrange(balltree, points[:,i], par, true) # usare un parametro abbastanza grande
		N = Common.neighborhood(kdtree,points,[i],Int[],threshold,k)
		centroid = Common.centroid(points[:,N])
		C = zeros(2,2)
		for j in N
			diff = points[:,j] - centroid
			C += diff*diff'
		end

		eigval = Lar.eigvals(C)
		curvature = eigval[1]/sum(eigval)
		curvs[i] = curvature
	end
	mu = StatsBase.mode(curvs)
	for i in 1:npoints
		if  curvs[i] > mu # TODO parametro da stimare in funzione dei dati.
			corners[i] = true
		end
	end

	return current_inds[corners], curvs
end
