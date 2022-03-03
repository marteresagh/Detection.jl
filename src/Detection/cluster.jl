"""
	get_hyperplane(params::Initializer; given_seed=nothing::Union{Nothing,Int64})

Return hyperplane indicated by seed (random or provided by user).
"""
function no_rand_get_hyperplane(params::Initializer; given_seed=nothing::Union{Nothing,Int64})

	if isnothing(given_seed) # se non è dato
		# 1. ricerca del seed random
		candidates = setdiff(params.current_inds,params.visited)
		possible_seeds = params.PC.coordinates[:,candidates] 	# qui gli indici sono relativi ai candidati
		init_seed = findall(x->x==candidates[1], params.current_inds)
		union!(params.visited,params.current_inds[init_seed])
		seed, hyperplane = seedpoint(possible_seeds, params; given_seed = 1)
		# da qui in poi indici relativi ai punti correnti
		R = findall(x->x==candidates[seed], params.current_inds)
		union!(params.visited,params.current_inds[R])
	else
		# 1. ricerca del seed partendo da uno dato
		points = params.PC.coordinates[:,params.current_inds]
		init_seed = findall(x->x == given_seed, params.current_inds)[1]
		seed, hyperplane = seedpoint(points, params; given_seed = init_seed)
		R = findall(x->x==params.current_inds[seed], params.current_inds)
	end

	# 2. criterio di crescita
	all_visited_verts = search_cluster(R, hyperplane, params) #punti che non devono far parte dei mie seeds
	listPoint = params.PC.coordinates[:,params.current_inds[R]]
	listRGB = params.PC.rgbs[:,params.current_inds[R]]
	hyperplane.inliers = PointCloud(listPoint,listRGB)


	# gli indici tornano relativi ai punti totali
	return hyperplane, params.current_inds[R], params.current_inds[all_visited_verts]
end


function get_hyperplane(params::Initializer; given_seed=nothing::Union{Nothing,Int64})

	if isnothing(given_seed) # se non è dato
		# 1. ricerca del seed random
		candidates = setdiff(params.current_inds,params.visited)
		possible_seeds = params.PC.coordinates[:,candidates]
		# indice random
		rand_idx = rand(1:size(possible_seeds,2))
		init_seed = findall(x->x==candidates[rand_idx], params.current_inds)
		union!(params.visited,params.current_inds[init_seed])
		# qui gli indici sono relativi ai candidati
		seed, hyperplane = seedpoint(possible_seeds, params; given_seed = rand_idx )
		# da qui in poi indici relativi ai punti correnti
		R = findall(x->x==candidates[seed], params.current_inds)
		union!(params.visited,params.current_inds[R])
	else
		# 1. ricerca del seed partendo da uno dato
		points = params.PC.coordinates[:,params.current_inds]
		init_seed = findall(x->x == given_seed, params.current_inds)[1]
		seed, hyperplane = seedpoint(points, params; given_seed = init_seed)
		R = findall(x->x==params.current_inds[seed], params.current_inds)
	end

	# 2. criterio di crescita
	all_visited_verts = search_cluster(R, hyperplane, params) #punti che non devono far parte dei mie seeds
	listPoint = params.PC.coordinates[:,params.current_inds[R]]
	listRGB = params.PC.rgbs[:,params.current_inds[R]]
	hyperplane.inliers = PointCloud(listPoint,listRGB)

	# gli indici tornano relativi ai punti totali
	return hyperplane, params.current_inds[R], params.current_inds[all_visited_verts]
end

"""
	search_cluster(PC::PointCloud, R::Array{Int64,1}, hyperplane::Hyperplane, params::Initializer)

Search of all points belonging to the cluster `R`.
"""
function search_cluster(R::Array{Int64,1}, hyperplane::Hyperplane, params::Initializer)

	PC = params.PC
	points = PC.coordinates[:,params.current_inds]
	normals = nothing
	if PC.dimension == 3
		normals = PC.normals[:,params.current_inds]
	end

	#kdtree = Search.BallTree(points)
	kdtree = Search.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[] # new seeds
		N = Search.n_inrange(kdtree,points,seeds,visitedverts,params.threshold)
		# N = Search.neighborhood(kdtree,points,seeds,visitedverts,params.threshold,params.k)
		union!(visitedverts,N)

		for i in N
			p = points[:,i]
			# metodo IN
			# if plane -> check normals
			if PC.dimension == 3
				# change direction change surface
				test_dist = residual(hyperplane)(p) < params.par
				test_normals = Common.abs(Common.dot(hyperplane.direction,normals[:,i])) >= 0.7
				if test_dist && test_normals
					push!(tmp,i)
					push!(R,i)
				end
			else
				if residual(hyperplane)(p) < params.par
					push!(tmp,i)
					push!(R,i)
				end
			end
		end

		listPoint = points[:,R]

		# metodo OUT
		todel = optimize!(points,R,hyperplane,params.par)
		seeds = setdiff(tmp,todel)
	end

	return visitedverts
end

"""
	optimize!(points::Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64)

Optimize hyperplane direction.
"""
function optimize!(points::Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64)
	# mean and std
	res = residual(hyperplane).([points[:,i] for i in R])
	mu = Statistics.mean(res)
	rho = Statistics.std(res)


	# remove points with large residue
	filter = [ res[i] <= mu for i in 1:length(res)  ]
	tokeep = R[filter]

	listPoint = points[:,tokeep]

	# update fit parameters
	direction, centroid = Common.LinearFit(listPoint)
	hyperplane.direction = direction
	hyperplane.centroid = centroid

	# elimino i punti che sono troppo distanti.
	res = residual(hyperplane).([points[:,i] for i in R])
	todel = [ res[i] > par/2 for i in 1:length(res) ]
	to_del = R[todel]
	setdiff!(R,to_del)
	# to_del = copy(R)
	return to_del
end


"""
	seedpoint(points::Points, params::Initializer; given_seed = rand(1:size(points,2))::Int64)

Return consinstent seed and fitted hyperplane.
"""
# init_seed = findall(x->x == given_seed, params.current_inds)[1] # se dato
function seedpoint(points::Points, params::Initializer; given_seed = rand(1:size(points,2))::Int64)

	kdtree = Search.KDTree(points)
	idxseeds = Search.neighborhood(kdtree,points,[given_seed],Int64[],params.threshold,params.k)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane
end

"""
	remove_points!(inds::Array{Int64,1},todel::Array{Int64,1})

Remove from inds the i-th element described in todel
"""
function remove_points!(inds::Array{Int64,1},todel::Array{Int64,1})
	setdiff!(inds,todel)
end

"""
	validity(hyperplane::Hyperplane, params::Initializer)

Assert the detected hyperplane is valid / interesting
"""
function validity(hyperplane::Hyperplane, params::Initializer)
	# VALIDITY
	pc_on_hyperplane = hyperplane.inliers
	@assert  pc_on_hyperplane.n_points > params.N "not valid"

	res = residual(hyperplane).([c[:] for c in eachcol(pc_on_hyperplane.coordinates)])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	@assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"  #0.005 che valore è?? come generalizzare??

end


"""
	residual(hyperplane::Union{Plane,Line})(point::Point)

Orthogonal distance `point` to `hyperplane`.
"""
function residual(hyperplane::Hyperplane)
	function residual0(point::Point)
		if length(point) == 2
			return Common.distance_point2line(hyperplane.centroid,hyperplane.direction)(point)
		elseif length(point) == 3
			return Common.distance_point2plane(hyperplane.centroid,hyperplane.direction)(point)
		end
	end
	return residual0
end

"""
Return index of point in points with minor residual.
"""
function minresidual(points::Points, hyperplane::Hyperplane)
	res = residual(hyperplane).( [c[:] for c in eachcol(points)])
	return findmin(res)[2]
end
