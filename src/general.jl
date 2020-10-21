function iterate_random_detection(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64)

	# 1. - initialization
	PCcurrent = deepcopy(PC)
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
				hyperplane = HyperplaneDetection(PCcurrent,par,threshold)
				validity(hyperplane,N) #validity gli passo l'iperpiano e
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
			deletePoints!(PCcurrent,hyperplane.points)
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
