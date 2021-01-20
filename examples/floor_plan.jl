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

PC = FileManager.las2pointcloud(fname)
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

# user - parameters
par = 0.07
failed = 100
N = 10
k = 30

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,k)

# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)

@time hyperplanes = Detection.iterate_detection(params,debug = true)

# hyperplanes,_,_ = Detection.get_hyperplane(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates)'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates[:,params.fitted])'),GL.COLORS[2]),
			GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),V),EV,GL.COLORS[1],1.0)
		])


GL.VIEW([	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates)'),GL.COLORS[2]),
			Visualization.mesh_lines(hyperplanes)...])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])
