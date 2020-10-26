function iterate_random_detection(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64)

	# 1. - initialization
	# if PC.dimension == 2
	# 	elimina vertici doppi
	# end

	current_inds = [1:PC.n_points...]
	hyperplanes = Hyperplane[]
	hyperplane = nothing
	cluster = nothing

	f = 0
	i = 0

	# find shapes
	flushprintln("======= Start search =======")
	search = true
	while search
		found = false

		while !found && f < failed
			try
				hyperplane, cluster = get_hyperplane_from_random_init_point(PC, current_inds, par, threshold)
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
			remove_points!(current_inds,cluster)
			# deletePoints!(PCcurrent,hyperplane.points)
		else
			search = false
		end

	end

	return hyperplanes, current_inds
end


function get_hyperplane_from_random_init_point(PC::PointCloud, current_inds::Array{Int64,1}, par::Float64, threshold::Float64)

	# firt sample
	points = PC.coordinates[:,current_inds]

	#da qui in poi indici relativi ai punti correnti
	index, hyperplane, first_index = seedpoint(points, threshold)
	R = [index]

	# search cluster
	hyperplane = search_cluster(points, R, hyperplane, par, threshold)

	listPoint = PC.coordinates[:,current_inds[R]]
	listRGB = PC.rgbs[:,current_inds[R]]
	hyperplane.points = PointCloud(listPoint,listRGB)

	return hyperplane, current_inds[R]
end


function search_cluster(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64, threshold::Float64)

	kdtree = Common.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[]
		N = Common.neighborhood(kdtree,points,seeds,visitedverts,threshold)

		for i in N
			p = points[:,i]
			if Common.residual(hyperplane)(p) < par
				push!(tmp,i)
				push!(R,i)
			end
			push!(visitedverts,i)
		end
		# == prova ad aggiungere qui l'eliminazione dei punti che hanno residuo troppo alto
		punti_da_buttare!(points, R, hyperplane, par)
		# ===
		listPoint = points[:,R]
		direction, centroid = Common.LinearFit(listPoint)
		hyperplane.direction = direction
		hyperplane.centroid = centroid
		seeds = tmp
	end


	return Hyperplane(hyperplane.direction, hyperplane.centroid)
end


function punti_da_buttare!(points::Lar.Points,R::Array{Int64,1},hyperplane::Hyperplane, par::Float64)
	res = Common.residual(hyperplane).([points[:,i] for i in R])

	mu = Statistics.mean(res)
	rho = Statistics.varm(res,mu)

	s = (res.-mu).^2

	filt = [s[i] < rho for i in 1:length(s)  ]

	R = R[filt]
end
