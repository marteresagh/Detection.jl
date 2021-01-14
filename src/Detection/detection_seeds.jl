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
			hyperplane, cluster, all_visited_verts = get_hyperplane(params,seed)
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
