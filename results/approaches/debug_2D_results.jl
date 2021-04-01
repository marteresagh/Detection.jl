using Detection
using Visualization
using Common
using AlphaStructures
using FileManager

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/NAVVIS"
INPUT_PC = FileManager.source2pc(source,4)

NAME_PROJ = "PLANE_NAVVIS_LOD4"

dirs = Detection.PlaneDirs( "D:/RISULTATI/TEST NAVVIS",NAME_PROJ)

hyperplanes = Hyperplane[]
for (root, dirs, files) in walkdir(dirs.PLANE)
	for file in files
	 	push!(hyperplanes,FileManager.load_hyperplane(joinpath(root,file)))
	end
end

#
# folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST/NAVVIS_LOD4"
# function create_folders(hyperplanes, folder_proj)
# 	for i in 1:length(hyperplanes)
# 		hyperplane = hyperplanes[i]
# 		folder = joinpath(folder_proj,"plane_$i")
# 		FileManager.mkdir_if(folder)
# 		inliers = hyperplane.inliers.coordinates
# 		hyperplane.inliers.rgbs = zeros(3,hyperplane.inliers.n_points)
# 		plane = Plane(hyperplane.direction, hyperplane.centroid)
#
# 		obb = Common.ch_oriented_boundingbox(inliers)
#
# 		extent = obb.scale
# 		center = obb.position
# 		euler = obb.rotation
#
# 		io = open(joinpath(folder,"finite_plane.txt"),"w")
#
# 		# plane
# 		write(io, "$(plane.a) $(plane.b) $(plane.c) $(plane.d)\n")
# 		# extent
# 		write(io, "$(extent[1]) $(extent[2]) $(extent[3])\n")
# 		# position
# 		write(io, "$(center[1]) $(center[2]) $(center[3])\n")
# 		# euler angles
# 		write(io, "$(euler[1]) $(euler[2]) $(euler[3])\n")
#
# 		close(io)
#
# 		FileManager.save_points_rgbs_txt(joinpath(folder,"inliers.txt"), hyperplane.inliers)
# 	end
# end
#
# create_folders(hyperplanes, folder_proj)

V,EV,FV = Common.DrawPlanes(hyperplanes; box_oriented=false)
centroid = Common.centroid(V)

GL.VIEW([
	# Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),EV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])


W = FileManager.load_points(joinpath(dirs.A_SHAPES,"a_shapes_points.txt"))
EW = FileManager.load_cells(joinpath(dirs.A_SHAPES,"a_shapes_edges.txt"))

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),W)'),GL.COLORS[2]),
	#Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
	#Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])
