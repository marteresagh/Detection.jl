"""
Find first seed randomly.
"""
function seedpoint(points::Lar.Points, params::Initializer, k=10::Int64)

	"""
	Return index of point in points with minor residual.
	"""
	function minresidual(points::Lar.Points, hyperplane::Hyperplane)
		res = Common.residual(hyperplane).([points[:,c] for c in 1:size(points,2)])
		return findmin(res)[2]
	end

	kdtree = Common.KDTree(points)
	randindex = rand(1:size(points,2))

	idxseeds = Common.neighborhood(kdtree,points,[randindex],Int64[],params.threshold)
	seeds = points[:,idxseeds]
	direction, centroid = Common.LinearFit(seeds)

	hyperplane = Hyperplane(direction,centroid)
	min_index = minresidual(seeds,hyperplane)
	seed = idxseeds[min_index]

	return seed, hyperplane
end

"""
remove from inds the i-th element described in todel
"""
function remove_points!(inds::Array{Int64,1},todel::Array{Int64,1})
	setdiff!(inds,todel)
end

"""
Assert the detected hyperplane is valid / interesting
"""
function validity(hyperplane::Hyperplane, params::Initializer)
	# VALIDITY
	pc_on_hyperplane = hyperplane.points
	#@show linedetected
	@assert  pc_on_hyperplane.n_points > params.N "not valid" #TODO da ottimizzare
	# line = linedetected.line
	# E,_ = PointClouds.DrawLine(pointsonline.points, line, 0.0)
	# dist = Lar.norm(E[:,1]-E[:,2])
	# rho = pointsonline.n/dist
	# PointClouds.flushprintln("rho = $rho")
	# @assert  rho > N "not valid"  #da automatizzare
end



# ==============  SAVES DONE
#
# function savePlanesDataset(planes::Array{PlaneDataset,1},params::PlaneDetectionParams)
# 	for i in 1:length(planes)
# 		filename = params.output+"/plane$i"
# 		savePlane(plane.plane,filename+."txt")
# 		savePoints(plane.points,filename*".las")
# 	end
# end
