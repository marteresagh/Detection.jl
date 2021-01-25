using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/las/wall.las"
fname = "examples/las/polyline.las"
fname = "examples/las/full.las"
fname = "examples/las/square.las"
fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"
fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/sezione_AMPHI_z39_5cm.las"
fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/casaletto_planimetria.las"

fname = "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS/SEZIONE_z=10_5.las"

PC = FileManager.las2pointcloud(fname)
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

# user - parameters
par = 0.05
failed = 100
N = 10
k = 30

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,2*k)

# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)

masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_sezione650.txt"
given_seeds = FileManager.load_points(masterseeds)
seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])
seeds = Int64[]

@time hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)

# hyperplanes,_,_ = Detection.get_hyperplane(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates)'),GL.COLORS[12]),
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates[:,params.fitted])'),GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),V),EV,GL.COLORS[1],1.0)
])


GL.VIEW([
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates)'),GL.COLORS[2]),
	Visualization.mesh_lines(hyperplanes)...
])


GL.VIEW([
	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
])
