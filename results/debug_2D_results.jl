using Detection
using Visualization
using Common
using AlphaStructures
using FileManager

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/NAVVIS"
INPUT_PC = FileManager.source2pc(source,4)

NAME_PROJ = "PLANE_NAVVIS_LOD4"

dirs = Detection.PlaneDirs( "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS",NAME_PROJ)

hyperplanes = Hyperplane[]
for (root, dirs, files) in walkdir(dirs.PLANE)
	for file in files
	 	push!(hyperplanes,FileManager.load_hyperplane(joinpath(root,file)))
	end
end


V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)
centroid = Common.centroid(V)

GL.VIEW([
	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
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
