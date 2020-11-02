function iterate_random_detection(params::Initializer)
	inputBuffer,task = monitorInput()
	# 1. - initialization
	hyperplanes = Hyperplane[]

	hyperplane = nothing
	cluster = nothing
	all_visited_verts = nothing

	f = 0
	i = 0

	# find shapes
	flushprintln("= Start search =")
	search = true
	while search

		if isready(inputBuffer) && take!(inputBuffer) == 'q'
	        break
	    end

		found = false
		while !found && f < params.failed
			try
				hyperplane, cluster, all_visited_verts = get_hyperplane_from_random_init_point(params)
				validity(hyperplane, params) #validity gli passo l'iperpiano e i parametri per la validità
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
			#remove_points!(params.current_inds,cluster) # nuovi punti di input
			union!(params.visited,all_visited_verts) # i punti su cui non devo provare a ricercare il seed
			union!(params.visited,cluster) # non li tolgo dal modello ma li marco come visitati
		else
			search = false
		end

	end

	try
	    Base.throwto(task, InterruptException())
	catch y
		continue
	end

	return hyperplanes
end


function get_hyperplane_from_random_init_point(params::Initializer)

	points = params.PC.coordinates[:,params.current_inds]

	# 1. ricerca del seed

	# qui gli indici sono relativi ai candidati
	candidates = setdiff(params.current_inds,params.visited)
	possible_seeds = params.PC.coordinates[:,candidates]
	index, hyperplane = seedpoint(possible_seeds, params)

	# da qui in poi indici relativi ai punti correnti
	R = findall(x->x==candidates[index], params.current_inds)

	# 2.  search cluster
	all_visited_verts = search_cluster(points, R, hyperplane, params) #punti che non devono far parte dei mie seeds
	listPoint = params.PC.coordinates[:,params.current_inds[R]]
	listRGB = params.PC.rgbs[:,params.current_inds[R]]
	hyperplane.points = PointCloud(listPoint,listRGB)

	# gli indici tornano relativi ai punti totali
	return hyperplane, params.current_inds[R], params.current_inds[all_visited_verts]
end


function search_cluster(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, params::Initializer)

	kdtree = Common.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[]
		N = Common.neighborhood(kdtree,points,seeds,visitedverts,params.threshold,params.k)
		for i in N
			p = points[:,i]
			if Common.residual(hyperplane)(p) < params.par
				push!(tmp,i)
				push!(R,i)
			end
			push!(visitedverts,i)
		end

		listPoint = points[:,R]
		direction, centroid = Common.LinearFit(listPoint)
		hyperplane.direction = direction
		hyperplane.centroid = centroid
		# seeds = tmp
		# == optimize da sistemare
		todel = optimize!(points,R,hyperplane,params.par) #TODO da sistemare questa cosa
		seeds = setdiff(tmp,todel)
		# ==
	end

	return visitedverts
end

#
# function punti_da_tenere!(points::Lar.Points, R::Array{Int64,1},hyperplane::Hyperplane)
#
# 	res = Common.residual(hyperplane).([points[:,i] for i in R])
# 	mu = Statistics.mean(res)
# 	rho = Statistics.std(res)
# 	todel = [mu - rho < res[i] < mu + rho for i in 1:length(res)  ]
#
# 	setdiff!(R, R[todel])
# end
#

function optimize!(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64)

	# prima parte
	res = Common.residual(hyperplane).([points[:,i] for i in R])
	mu = Statistics.mean(res)
	rho = Statistics.std(res)

	filter = [mu - rho < res[i] < mu + rho for i in 1:length(res)  ]
	tokeep = R[filter]

	listPoint = points[:,tokeep]
	direction, centroid = Common.LinearFit(listPoint)
	hyperplane.direction = direction
	hyperplane.centroid = centroid

	# seconda parte
	res = Common.residual(Hyperplane(direction,centroid)).([points[:,i] for i in R])
	todel = [ res[i] > par/2 for i in 1:length(res) ] #TODO da ottimizzare
	to_del = R[todel]
	setdiff!(R,to_del)

	return to_del
end
