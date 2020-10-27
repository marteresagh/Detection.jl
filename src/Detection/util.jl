"""
Delete points from model.
"""
function deletePoints!(PC::PointCloud, todel::PointCloud)
	tokeep = setdiff([1:PC.n_points...],[Common.matchcolumn(todel.coordinates[:,i], PC.coordinates) for i in 1:todel.n_points])

	coordinates = PC.coordinates[:,tokeep]
	rgbs = PC.rgbs[:,tokeep]
	PC = PointCloud(coordinates,rgbs)
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



# ==============  SAVES DONE
#
# function savePlanesDataset(planes::Array{PlaneDataset,1},params::PlaneDetectionParams)
# 	for i in 1:length(planes)
# 		filename = params.output+"/plane$i"
# 		savePlane(plane.plane,filename+."txt")
# 		savePoints(plane.points,filename*".las")
# 	end
# end
