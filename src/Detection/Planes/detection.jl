# TODO modificare i salvataggi
# per ogni shape creare una cartella_timestamp con :
# 1. file per descrizione piano infinito : normal e centroide
# 2. bounding box orientato
# 3. segmenti del bordo alpha shape
# 4. inliers

"""
	iterate_planes_detection(params::Initializer; seeds = Int64[]::Array{Int64,1}, debug = false)

Return hyperplanes in point cloud.

Algorithm description:
 - Search starts with initial parameters
 - Detects a valid hyperplane
 - If found, marks all vertices in cluster as visited
 - If not found, repeats the detection
 - Search terminates if the detection failed a number of times in a row
"""
function iterate_planes_detection(params::Initializer, output_folder::String; seeds = Int64[]::Array{Int64,1}, debug = false)
	inputBuffer,task = monitorInput() # premere 'q' se si vuole uscire dal loop senza perdere i dati

	# 1. - Initialization
	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	f = 0 # number of failed
	i = 0 # number of hyperplane found

	# 2. - Main loop
	println("= Start search =")

	for seed in seeds
		found = false
		try
			hyperplane, cluster, all_visited_verts = get_hyperplane(params; given_seed = seed)
			found = true
		catch y

		end

		if found
			i = i+1

			####################################
			timestamp = FileManager.Dates.datetime2epochms(Dates.now())
			folder = joinpath(output_folder,"plane_$timestamp")
			FileManager.mkdir_if(folder)
			save_finite_plane(folder, hyperplane)

			####################################

			println("$i of $(length(seeds))")
			union!(params.fitted,cluster)
			remove_points!(params.current_inds,cluster) # tolgo i punti dal modello
			union!(params.visited,all_visited_verts)
		end
	end

	flush(stdout)

	search = true
	while search

		if isready(inputBuffer) && take!(inputBuffer) == 'q'
			break # break main loop
		end

		found = false
		while !found && f < params.failed
			try
				hyperplane, cluster, all_visited_verts = get_hyperplane(params)
				union!(params.visited,all_visited_verts)
				validity(hyperplane, params) # test of validity
				found = true
			catch y
				f = f+1
				if f%10 == 0
					println("failed = $f")
				end
			end
		end

		if found
			f = 0
			i = i+1
			if i%10 == 0
				println("$i shapes found")
				flush(stdout)
			end

			####################################
			timestamp = FileManager.Dates.datetime2epochms(Dates.now())
			folder = joinpath(output_folder,"plane_$timestamp")
			FileManager.mkdir_if(folder)
			save_finite_plane(folder, hyperplane)

			####################################

			union!(params.fitted,cluster)
			remove_points!(params.current_inds,cluster) # tolgo i punti dal modello
			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed
		else
			search = false
		end

	end

	if debug # interrompe il task per la lettura da tastiera
		try
			Base.throwto(task, InterruptException())
		catch y
			println("STOPPED")
		end
	end

	return i
end
