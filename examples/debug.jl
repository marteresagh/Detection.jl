using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/las/wall.las"
fname = "examples/las/muriAngolo.las"
fname = "examples/las/area.las"
fname = "examples/las/colonna.las"
#fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"
#fname = "C:/Users/marte/Documents/GEOWEB/FilePotree/AMPHI/sezione_AMPHI_z39_5cm.las"

PC = FileManager.las2pointcloud(fname)
par = 0.07
threshold = 2*0.03
failed = 100
N = 5
INPUT_PC = PointCloud(PC.coordinates[1:2,:],PC.rgbs)
k = 20
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params,debug = true)

# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])


GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),	Visualization.mesh_lines(hyperplanes)...])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])
