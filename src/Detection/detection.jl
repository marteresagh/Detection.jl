function iterate_random_detection(params::initParams)

	# 1. - initialization
	# if PC.dimension == 2
	# 	elimina vertici doppi
	# end

	#current_inds = [1:PC.n_points...]
	hyperplanes = Hyperplane[]
	hyperplane = nothing
	cluster = nothing
	#visited = Int64[]
	visits = nothing
	f = 0
	i = 0

	# find shapes
	flushprintln("======= Start search =======")
	search = true
	while search
		found = false
		@show length(visited)
		while !found && f < params.failed
			try
				hyperplane, cluster, visits = get_hyperplane_from_random_init_point(params)#PC, current_inds, par, threshold, visited)
				validity(hyperplane, params.N) #validity gli passo l'iperpiano e i parametri per la validità
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
			union!(params.visited,visits)
			# deletePoints!(PCcurrent,hyperplane.points)
		else
			search = false
		end

	end

	return hyperplanes, current_inds, visited
end


function get_hyperplane_from_random_init_point(params::initParams)#PC::PointCloud, current_inds::Array{Int64,1}, par::Float64, threshold::Float64, visited::Array{Int64,1})

	# firt sample
	points = params.PC.coordinates[:,params.current_inds]
	not_visited = setdiff(params.current_inds,params.visited)
	#da qui in poi indici relativi ai punti correnti
	index, hyperplane, first_index = seedpoint(points, params.threshold, not_visited)
	R = [index]

	# search cluster
	visited = search_cluster(points, R, hyperplane, par, threshold)
	listPoint = PC.coordinates[:,current_inds[R]]
	listRGB = PC.rgbs[:,current_inds[R]]
	hyperplane.points = PointCloud(listPoint,listRGB)


	return hyperplane, current_inds[R], current_inds[visited]
end


function search_cluster(points::Lar.Points, R::Array{Int64,1}, hyperplane::Hyperplane, params::initParams)

	kdtree = Common.KDTree(points)
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[]
		N = Common.neighborhood(kdtree,points,seeds,visitedverts,params.threshold)

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
		seeds = tmp
		# == optimize da sistemare
		optimize!(points,R,hyperplane,params.par)
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
	todel = [ res[i] > par/2 for i in 1:length(res) ]
	setdiff!(R,R[todel])

	#return direction, centroid
end
