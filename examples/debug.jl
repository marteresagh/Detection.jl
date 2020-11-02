using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"

#######################  REMOVE POINTS
fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z650.las"#"C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\Sezione_z39_10cm.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
current_inds = [1:PC2D.n_points...]
k = 20

outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),

		])

par = 0.07
threshold = 2*0.03
failed = 10
N = 100

params = Initializer(PC2D,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])
