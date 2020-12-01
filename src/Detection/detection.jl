"""
iterate_random_detection(params::Initializer; debug = false)

Return all found hyperplanes.

Algorithm description:
 - Search starts with initial parameters
 - Detects a valid hyperplane
  - If found, marks all verteces in cluster as visited
  - If not found, repeats the detection
 - Search terminates if the detection failed a number of times in a row
"""
function iterate_random_detection(params::Initializer; debug = false)
	inputBuffer,task = monitorInput() # premere 'q' se si vuole uscire dal loop senza perdere i dati

	# 1. - Initialization
	hyperplanes = Hyperplane[]

	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	f = 0 # number of failed
	i = 0 # number of hyperplane found

	# 2. - Main loop
	flushprintln("= Start search =")
	search = true
	while search

		if isready(inputBuffer) && take!(inputBuffer) == 'q'
			break # break main loop
		end

		found = false
		while !found && f < params.failed
			try
				hyperplane, cluster, all_visited_verts = get_hyperplane_from_random_init_point(params)
				validity(hyperplane, params) # test of validity
				found = true
			catch y
				f = f+1
				if f%10 == 0
					flushprintln("failed = $f")
				end
			end
		end

		if found
			f = 0
			i = i+1
			if i%10 == 0
				flushprintln("$i shapes found")
			end
			push!(hyperplanes,hyperplane)
			union!(params.fitted,cluster)
			# remove_points!(params.current_inds,cluster) # tolgo i punti dal modello
			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed
		else
			search = false
		end

	end

	if debug # interrompe il task per la lettura da tastiera
		try
			Base.throwto(task, InterruptException())
		catch y
			flushprintln("interrotto")
		end
	end

	return hyperplanes
end

"""
get_hyperplane_from_random_init_point(params::Initializer)

Detect an hyperplane starting from initial random point.
"""
function get_hyperplane_from_random_init_point(params::Initializer)

	points = params.PC.coordinates[:,params.current_inds]

	# 1. ricerca del seed
	candidates = setdiff(params.current_inds,params.visited)
	possible_seeds = params.PC.coordinates[:,candidates] 	# qui gli indici sono relativi ai candidati
	index, hyperplane = seedpoint(possible_seeds, params)

	# da qui in poi indici relativi ai punti correnti
	R = findall(x->x==candidates[index], params.current_inds)

	# 2. criterio di crescita
	all_visited_verts = search_cluster(points, R, hyperplane, params) #punti che non devono far parte dei mie seeds
	listPoint = params.PC.coordinates[:,params.current_inds[R]]
	listRGB = params.PC.rgbs[:,params.current_inds[R]]
	hyperplane.inliers = PointCloud(listPoint,listRGB)

	# gli indici tornano relativi ai punti totali
	return hyperplane, params.current_inds[R], params.current_inds[all_visited_verts]
end

"""
Search of all points belonging to the cluster `R`.
"""
function search_cluster(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, params::Initializer)

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
			if size(points,1) == 3
				K = Common.neighborhood(kdtree,points,[i],Int[],params.threshold,params.k)
				normal,_ = Common.LinearFit(points[:,K])
				test_normals = angle_between_vectors(hyperplane.direction,normal) <= pi/4
				if Common.residual(hyperplane)(p) < params.par && test_normals
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
		# optimize!(points,R,hyperplane,params.par)
		# seeds = tmp
	end

	return visitedverts
end

"""

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
