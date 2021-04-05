using Common
using FileManager
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/CASALETTO"
INPUT_PC = FileManager.source2pc(source,0)

centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "CASALETTO_LOD3"
folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"

folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)

# hyperplanes, _ = FileManager.get_hyperplanes(folders)
# V,EV,FV = Common.DrawPlanes(hyperplanes; box_oriented=false)
#
# GL.VIEW([
# #	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
# ])
#
# GL.VIEW([
# 	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
# ])


function get_boundary_models(folders)
	n_planes = length(folders)
	boundary = Lar.LAR[]
	for i in 1:n_planes
	#	println("$i of $n_planes")
		if isfile(joinpath(folders[i],"execution.probe"))
			V = FileManager.load_points(joinpath(folders[i],"boundary_points3D.txt"))
			EV = FileManager.load_connected_components(joinpath(folders[i],"boundary_edges.txt"))
			model = (V,EV)
			push!(boundary,model)
		end
	end
	return boundary
end

boundary_models = get_boundary_models(folders)

GL.VIEW([
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates))),
	[GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),model[1]),model[2],GL.COLORS[rand(1:1)],0.8) for model in boundary_models]...,
])


s = 0
for model in boundary_models
	global s
	s += size(model[1],2)
end


# #############################  MERGE PLANE  ##################################################
# output_folder = "C:/Users/marte/Documents/GEOWEB/TEST/CASALETTO_MERGE"
#
# function merge_plane(folders, output_folder)
#
# 	function point_cloud_distance(source::Lar.Points, target::Lar.Points)
# 		kdtree = Common.KDTree(target)
# 		idxs, dists = Common.NearestNeighbors.nn(kdtree, source)
# 		return idxs,dists
# 	end
#
# 	n_planes = length(folders)
# 	hyperplanes, _ = FileManager.get_hyperplanes(folders)
# 	dict = DataStructures.Dict()
# 	for i in 1:n_planes
# 		@show "giro",i
# 		hyperplane = hyperplanes[i]
# 		dir = hyperplane.direction
# 		cen = Common.centroid(hyperplane.inliers.coordinates)
# 		inliers = hyperplane.inliers.coordinates
#
# 		key_ = (dir,cen)
# 		for key in keys(dict)
#
# 			test_angle = Common.angle_between_directions(dir,key[1]) < pi/8
# 			test_dist_centroid = Common.Dist_Point2Plane(cen,Hyperplane(key...)) < 0.01
# 			idxs,dists = point_cloud_distance(inliers,dict[key])
# 			test_dist_pcs = min(dists...) < 0.1
# 		# 	@show test_angle
# 		# #	@show test_dist_centroid
# 		# 	@show test_dist_pcs
# 			if test_angle && test_dist_centroid && test_dist_pcs
# 				key_ = key
# 				break
# 			end
# 		end
#
# 		if !haskey(dict,key_)
# 			dict[key_] = inliers
# 		else
# 			dict[key_] = hcat(dict[key_],inliers)
# 		end
#
# 	end
# 	return dict
# end
#
# dict = merge_plane(folders, output_folder)
#
# hyperplanes = Hyperplane[]
# for key in keys(dict)
# 	inliers = dict[key]
# 	params = Common.Fit_Plane(inliers)
# 	push!(hyperplanes, Hyperplane(PointCloud(inliers), params...))
# end
#
#
# V,FV = Common.DrawPlanes(hyperplanes; box_oriented = false)
#
# GL.VIEW([
# #	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
# ])
#
# ###############################################################################
