using Detection
using Visualization
using Common
using AlphaStructures
using FileManager

dirs = Detection.PlaneDirs( "C:/Users/marte/Documents/GEOWEB/TEST","PLANE_CASALETTO")

hyperplanes = Hyperplane[]
for (root, dirs, files) in walkdir(dirs.PLANE)
	for file in files
	 	push!(hyperplanes,FileManager.load_hyperplane(joinpath(root,file)))
	end
end

V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)
centroid = Common.centroid(V)

GL.VIEW([
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])

W,EW = save_alpha_shape_model(hyperplanes, joinpath(dirs.A_SHAPES,"a_shapes"))
W = FileManager.load_points(joinpath(dirs.A_SHAPES,"a_shapes_points.txt"))
EW = FileManager.load_cells(joinpath(dirs.A_SHAPES,"a_shapes_edges.txt"))

L,EL = Detection.linearization(W,EW)

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),W)'),GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],1.0),
	# GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),L)'),GL.COLORS[12]),
	# GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),L),EL,GL.COLORS[rand(1:12)],1.0),
])
