"""
	iterate_detection(params::Initializer; seeds = Int64[]::Array{Int64,1}, debug = false)

Return hyperplanes in point cloud.

Algorithm description:
 - Search starts with initial parameters
 - Detects a valid hyperplane
 - If found, marks all vertices in cluster as visited
 - If not found, repeats the detection
 - Search terminates if the detection failed a number of times in a row
"""
function iterate_detection(params::Initializer; seeds = Int64[]::Array{Int64,1}, debug = false)
	#inputBuffer,task = monitorInput() # premere 'q' se si vuole uscire dal loop senza perdere i dati

	# 1. - Initialization
	hyperplanes = Hyperplane[]
	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	f = 0 # number of failed
	i = 0 # number of hyperplane found

	# 2. - Main loop
	@info("= Start search =")

	for seed in seeds
		found = false
		try
			hyperplane, cluster, all_visited_verts = get_hyperplane(params; given_seed = seed)
			found = true
		catch y

		end

		if found
			i = i+1
			@debug("$i of $(length(seeds))")
			push!(hyperplanes,hyperplane)
			union!(params.fitted,cluster)

			if params.PC.dimension==3
				 remove_points!(params.current_inds,cluster) # tolgo i punti dal modello
			end

			union!(params.visited,all_visited_verts)
			if isempty(setdiff(params.current_inds,params.visited))
				break
			end

			# save_plane() |_ save_hyperplane() attenzionecon le linee e con i piani
			# save_lines() |

		end
	end

	search = true
	while search


		#
		# if isready(inputBuffer) && take!(inputBuffer) == 'q'
		# 	break # break main loop
		# end

		found = false
		nomorepoints = false
		while !found && f < params.failed && !nomorepoints

			 try
				hyperplane, cluster, all_visited_verts = get_hyperplane(params)
				validity(hyperplane, params) # test of validity
				found = true
			catch e
				# println(e)
				# @error "Something went wrong" exception = (e, catch_backtrace())
			 	f = f+1
				if f%10 == 0
					@debug("failed = $f")
				end

				if isempty(setdiff(params.current_inds,params.visited))
					nomorepoints = true
				end

			end
		end

		if found
			# save_plane() |_ save_hyperplane() attenzionecon le linee e con i piani
			# save_lines() |
			f = 0
			i = i+1
			if i%10 == 0
				@debug("$i shapes found")
			end
			push!(hyperplanes,hyperplane)
			union!(params.fitted,cluster)

			if params.PC.dimension==3
			 	 remove_points!(params.current_inds,cluster) # tolgo i punti dal modello
			 end

			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed

		else
			search = false
		end

	end

	# if debug # interrompe il task per la lettura da tastiera
	# 	try
	# 		Base.throwto(task, InterruptException())
	# 	catch y
	# 		println("STOPPED")
	# 	end
	# end

	return hyperplanes
end
