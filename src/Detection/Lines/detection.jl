"""
	iterate_lines_detection(
		params::Initializer,
		affine_matrix::Matrix,
		s_2d::IOStream,
		s_3d::IOStream;
		seeds = Int64[]::Array{Int64,1},
		debug = false
		)

Detect lines in 2D point cloud.

Algorithm description:
 - Search starts with initial parameters
 - A line is detected
 - If valid, all vertices in cluster are marked as visited and segment are saved in a text file
 - If not valid, the detection is repeated
 - Search ends if the detection failed a number of times in a row
"""
function iterate_lines_detection(
	params::Initializer,
	affine_matrix::Matrix,
	s_2d::IOStream,
	s_3d::IOStream;
	seeds = Int64[]::Array{Int64,1},
	debug = false
	)

	inputBuffer,task = monitorInput() # premere 'q' se si vuole uscire dal loop senza perdere i dati

	# 1. - Initialization
	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	f = 0 # number of failed
	i = 0 # number of hyperplane found

	# 2. - Main loop
	flushprintln("= Start search =")

	for seed in seeds
		found = false
		try
			hyperplane, cluster, all_visited_verts = get_hyperplane(params; given_seed = seed)
			found = true
		catch y

		end

		if found
			i = i+1
			flushprintln("$i of $(length(seeds))")

			write_line(s_2d, s_3d, line, affine_matrix)

			union!(params.fitted,cluster)
			union!(params.visited,all_visited_verts)

		end
	end

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
					flushprintln("failed = $f")
				end
			end
		end

		if found
			f = 0
			i = i+1
			if i%10 == 0
				flushprintln("$i lines detected")
			end

			write_line(s_2d, s_3d, line, affine_matrix)

			union!(params.fitted,cluster)
			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed

		else
			search = false
		end

	end

	if debug # interrompe il task per la lettura da tastiera
		try
			Base.throwto(task, InterruptException())
		catch y
			flushprintln("STOPPED")
		end
	end

	return i
end
