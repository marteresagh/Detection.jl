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
"""
function get_hyperplane(params::Initializer; given_seed=nothing::Union{Nothing,Int64})

	if isnothing(given_seed) # se non è dato
		# 1. ricerca del seed random
		candidates = setdiff(params.current_inds,params.visited)
		possible_seeds = params.PC.coordinates[:,candidates] 	# qui gli indici sono relativi ai candidati
		seed, hyperplane = seedpoint(possible_seeds, params)
		# da qui in poi indici relativi ai punti correnti
		R = findall(x->x==candidates[seed], params.current_inds)
	else
		# 1. ricerca del seed partendo da uno dato
		points = params.PC.coordinates[:,params.current_inds]
		init_seed = findall(x->x == given_seed, params.current_inds)[1]
		seed, hyperplane = seedpoint(points, params, given_seed)
		R = findall(x->x==params.current_inds[seed], params.current_inds)
	end

	# 2. criterio di crescita
	all_visited_verts = search_cluster(params.PC, R, hyperplane, params) #punti che non devono far parte dei mie seeds
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
function search_cluster(PC::PointCloud, R::Array{Int64,1}, hyperplane::Hyperplane, params::Initializer)

	points = params.PC.coordinates[:,params.current_inds]
	normals = nothing
	if PC.dimension == 3
		normals = params.PC.normals[:,params.current_inds]
	end

	kdtree = Common.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[] # new seeds
		N = Common.neighborhood(kdtree,points,seeds,visitedverts,params.threshold,params.k)
		union!(visitedverts,N)

		for i in N
			p = points[:,i]
			# metodo IN
			# if plane -> check normals
			if PC.dimension == 3
				# change direction change surface
				test_dist = Common.residual(hyperplane)(p) < params.par
				test_normals = Common.angle_between_directions(hyperplane.direction,normals[:,i]) <= pi/4
				if test_dist && test_normals
					push!(tmp,i)
					push!(R,i)
				end
			else
				if Common.residual(hyperplane)(p) < params.par
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
	optimize!(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64)
"""
function optimize!(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64)
	# mean and std
	res = Common.residual(hyperplane).([points[:,i] for i in R])
	mu = Statistics.mean(res)
	rho = Statistics.std(res)

	# remove points with large residue
	filter = [ res[i] < mu for i in 1:length(res)  ]
	tokeep = R[filter]

	listPoint = points[:,tokeep]

	# update fit parameters
	direction, centroid = Common.LinearFit(listPoint)
	hyperplane.direction = direction
	hyperplane.centroid = centroid

	# elimino i punti che sono troppo distanti.
	res = Common.residual(hyperplane).([points[:,i] for i in R])
	todel = [ res[i] > par/2 for i in 1:length(res) ]
	to_del = R[todel]
	setdiff!(R,to_del)

	return to_del
end
