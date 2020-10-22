function iterate_random_detection(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64)

	# 1. - initialization
	#PCcurrent = deepcopy(PC)
	#gia da qui ho creato PC con punti 2D o 3D
	#prova ad usare solo gli indici
	# if PC.dimension == 2
	# 	elimina vertici doppi
	# end

	currents_inds = [1:PC.n_points...]
	hyperplanes = Hyperplane[]
	hyperplane = nothing
	R = nothing

	f = 0
	i = 0

	# find shapes
	flushprintln("======= Start search =======")
	search = true
	while search
		found = false

		while !found && f < failed
			try
				hyperplane, R = get_hyperplane_from_random_init_point(PC, currents_inds, par, threshold)
				validity(hyperplane, N) #validity gli passo l'iperpiano e i parametri per la validità
				found = true
			catch y
				f = f+1
				flushprintln("failed = $f")
			end
		end

		if found
			f = 0
			i = i+1
			flushprintln("$i shapes found")
			push!(hyperplanes,hyperplane)
			remove_points!(currents_inds,R)
			# deletePoints!(PCcurrent,hyperplane.points)
		else
			search = false
		end

	end

	return hyperplanes, currents_inds
end


function get_hyperplane_from_random_init_point(PC::PointCloud, currents_inds::Array{Int64,1}, par::Float64, threshold::Float64)

	# firt sample
	flushprintln("entro")
	index, hyperplane, randindex = seedpoint(PC,currents_inds, threshold)
	R = [index]

	# search cluster
	hyperplane = search_cluster(PC, R, hyperplane, par, threshold)

	return hyperplane, currents_inds[R]
end


function search_cluster(PC::PointCloud, currents_inds::Array{Int64,1}, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64, threshold::Float64)
	points = PC.coordinates[:,currents_inds]
	kdtree = Common.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[]
		N = Common.neighborhood(kdtree,PC,seeds,visitedverts,threshold)

		for i in N
			p = PC.points[:,currents_inds[i]]
			if Common.residual(p,hyperplane) < par
				push!(tmp,i)
				push!(R,i)
			end
			push!(visitedverts,i)
		end

		listPoint = PC.points[:,currents_inds[R]]
		direction, centroid = Common.LinearFit(listPoint)
		hyperplane.direction = direction
		hyperplane.centroid = centroid
		seeds = tmp
	end

	listRGB = PC.rgbs[:,currents_inds[R]]
	return Hyperplane(PointCloud(listPoint, listRGB), direction, centroid), currents_inds[R]
end
