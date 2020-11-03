using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"

#######################  REMOVE POINTS
"C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z650.las"
fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\Sezione_z39.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

points, indx = Common.remove_double_verts(PC.coordinates[1:2,:],2)
current_inds = collect(1:PC2D.n_points)
k = 20

outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,indx]'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),

		])

par = 0.02
threshold = 2*0.2
failed = 1000
N = 100
INPUT_PC = PointCloud(PC.coordinates[1:2,indx],PC.rgbs[:,indx])
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params)

hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])

GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,params.fitted]'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])
