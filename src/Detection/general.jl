function iterate_random_detection(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64)

	# 1. - initialization
	#PCcurrent = deepcopy(PC)
	#gia da qui ho creato PC con punti 2D o 3D
	#prova ad usare solo gli indici
	# if PC.dimension == 2
	# 	elimina vertici doppi
	# end

	currents_inds = [1:PC.n_points...]
	hyperplanes = Hyperplanes[]
	hyperplane = nothing

	f = 0
	i = 0

	# find shapes
	flushprintln("======= Start search =======")
	search = true
	while search
		found = false

		while !found && f < failed
			try
				hyperplane,R = get_hyperplane_from_random_init_point(PC, currents_inds, par, threshold)

				validity(hyperplane, N) #validity gli passo l'iperpiano e
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

	return hyperplanes
end

"""
Assert the detected hyperplane is valid / interesting
"""
function validity(hyperplane::Hyperplane,N::Int64)
	# VALIDITY
	pc_on_hyperplane = hyperplane.points
	#@show linedetected
	@assert  pc_on_hyperplane.n_points > N "not valid"  #da automatizzare
	# line = linedetected.line
	# E,_ = PointClouds.DrawLine(pointsonline.points, line, 0.0)
	# dist = Lar.norm(E[:,1]-E[:,2])
	# rho = pointsonline.n/dist
	# PointClouds.flushprintln("rho = $rho")
	# @assert  rho > N "not valid"  #da automatizzare
end



"""
Delete points from model.
"""
function deletePoints!(PC::PointCloud, todel::PointCloud)
	tokeep = setdiff([1:PC.n_points...],[Common.matchcolumn(todel.coordinates[:,i], PC.coordinates) for i in 1:todel.n_points])

	coordinates = PC.coordinates[:,tokeep]
	rgbs = PC.rgbs[:,tokeep]
	PC = PointCloud(coordinates,rgbs)
end

function remove_points!(currents_inds::Array{Int64,1},R::Array{Int64,1})
	setdiff!(currents_inds,R)
end

function get_hyperplane_from_random_init_point(PC::PointCloud, currents_inds::Array{Int64,1}, par::Float64, threshold::Float64)

	# firt sample
	index, hyperplane, randindex = seedpoint(PC.points[:,currents_inds], threshold)
	R = [index]

	# search cluster
	hyperplane = search_cluster(PC, R, hyperplane, par, threshold)

	return hyperplane, currents_inds[randindex]
end


function search_cluster(PC::PointCloud, currents_inds::Array{Int64,1}, R::Array{Int64,1}, hyperplane::Hyperplane, par::Float64, threshold::Float64)
	kdtree = KDTree(PC.points[:,currents_inds])
	seeds = copy(R)
	visitedverts = copy(R)
	listPoint = nothing

	while !isempty(seeds)
		tmp = Int[]
		N = PointClouds.neighborhood(kdtree,PC,seeds,visitedverts,threshold)

		for i in N
			p = PC.points[:,i]
			if residual(p,hyperplane) <= par
				push!(tmp,i)
				push!(R,i)
			end
			push!(visitedverts,i)
		end

		listPoint = PC.points[:,R]
		direction, centroid = Common.Linear_fit(listPoint)
		hyperplane.direction = direction
		hyperplane.centroid = centroid
		seeds = tmp
		#setdiff!(seeds,seed)
	end
	listRGB = PC.rgbs[:,R]
	return Hyperplane(PointCloud(length(R), listPoint, listRGB), direction, centroid),R
end

"""
"""
function get_hyperplane(PC, currents_inds, par, threshold)
	if PC.dimension == 3
		return random_plane(PC, currents_inds, par, threshold)
	elseif PC.dimension == 2
		return random_line(PC, currents_inds, par, threshold)
	end
end
