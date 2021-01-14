function iterate_seeds_detection(params::Initializer, seeds::Array{Int64,1}; debug = false)
	inputBuffer,task = monitorInput() # premere 'q' se si vuole uscire dal loop senza perdere i dati

	# 1. - Initialization
	hyperplanes = Hyperplane[]

	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	i = 0 # number of hyperplane found

	# 2. - Main loop
	flushprintln("= Start search =")
	for seed in seeds
		found = false

		if isready(inputBuffer) && take!(inputBuffer) == 'q'
			break # break main loop
		end

		try
			hyperplane, cluster, all_visited_verts = get_hyperplane_from_seed(params,seed)
			found = true
		catch y
		end

		if found
			i = i+1
			flushprintln("$i shapes found of $(length(seeds))")
			push!(hyperplanes,hyperplane)
			union!(params.fitted,cluster)
			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed
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

#
# function get_hyperplane_from_seed(params::Initializer, given_seed::Int64)
#
# 	# 1. ricerca del seed partendo da uno dato
# 	points = params.PC.coordinates[:,params.current_inds]
# 	init_seed = findall(x->x == given_seed, params.current_inds)[1]
# 	seed, hyperplane = seedpoint(points, params, given_seed)
#
# 	R = findall(x->x==params.current_inds[seed], params.current_inds)
#
# 	# 2. criterio di crescita
# 	all_visited_verts = search_cluster(params.PC, R, hyperplane, params) #punti che non devono far parte dei mie seeds
# 	listPoint = params.PC.coordinates[:,params.current_inds[R]]
# 	listRGB = params.PC.rgbs[:,params.current_inds[R]]
# 	hyperplane.inliers = PointCloud(listPoint,listRGB)
#
# 	# gli indici tornano relativi ai punti totali
# 	return hyperplane, params.current_inds[R], params.current_inds[all_visited_verts]
# end


function get_hyperplane(params::Initializer; given_seed=nothing::Union{Nothing,Int64})

	if isnothing(given_seed) # se non è dato
		# 1. ricerca del seed random
		candidates = setdiff(params.current_inds,params.visited)
		possible_seeds = params.PC.coordinates[:,candidates] 	# qui gli indici sono relativi ai candidati
		seed, hyperplane = seedpoint(possible_seeds, params)
		# da qui in poi indici relativi ai punti correnti
		R = findall(x->x==candidates[index], params.current_inds)
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
