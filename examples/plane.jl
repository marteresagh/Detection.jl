using Common
using FileManager
using Features
using Search
using Detection
using Visualization
using AlphaStructures

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/COLONNA"
INPUT_PC = FileManager.source2pc(source,1)

# user parameters
par = 0.07
failed = 100
N = 40
k = 30

# threshold estimation
threshold = Features.estimate_threshold(INPUT_PC,2*k)

# normals
normals = Features.compute_normals(INPUT_PC.coordinates, threshold, k)
INPUT_PC.normals = normals

# seeds indices
# masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_COLOMBELLA.txt"
# given_seeds = FileManager.load_points(masterseeds)
# seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

seeds = Int64[]
# outliers
outliers = Features.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)

# 2. Detection
hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)
hyperplane, cluster, all_visited_verts = Detection.get_hyperplane(params)
centroid = Common.centroid(INPUT_PC.coordinates)

V,FV = Common.DrawPlanes(hyperplanes; box_oriented = false)

GL.VIEW([
	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates[:,outliers])'),GL.COLORS[2]) ,
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])
