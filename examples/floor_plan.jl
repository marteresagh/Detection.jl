using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/las/wall.las"
fname = "examples/las/polyline.las"
fname = "examples/las/full.las"
fname = "examples/las/square.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z250.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/sezione_AMPHI_z39_5cm.las"

PC = FileManager.las2pointcloud(fname)
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

# user - parameters
par = 0.07
failed = 100
N = 50
k = 20

# threshold estimation
threshold = Detection.estimate_threshold(INPUT_PC,k)

# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params,debug = true)

# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])


GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(hyperplanes)...])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])
