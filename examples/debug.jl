using Detection
using Visualization
using Common
using FileManager
using Statistics

#fname = "examples/wall.las"
#fname = "examples/muriAngolo.las"
#fname = "examples/area.las"
fname = "C:/Users/marte/Documents/GEOWEB/FilePotree/orthoCONTEA/Sezione_z650.las"
#fname = "C:/Users/marte/Documents/GEOWEB/FilePotree/AMPHI/sezione_AMPHI_z39_5cm.las"

PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
# points, indx = Common.remove_double_verts(PC.coordinates[1:2,:],2)
# current_inds = collect(1:PC2D.n_points)
k = 20

outliers = Common.outliers(PC2D, collect(1:PC2D.n_points), k)
# da_tenere = setdiff(current_inds,outliers)

# GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,:]'),GL.COLORS[2]) ,
# 			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
#   			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),
#
# 		])

par = 0.07
threshold = 2*0.03
failed = 100
N = 10
INPUT_PC = PointCloud(PC.coordinates[1:2,:],PC.rgbs)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params)

# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,:]'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])
#
# GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,params.fitted]'),GL.COLORS[2]) ,
# 			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
#   			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
# 		])
#

source = "C:\\Users\\marte\\Documents\\potreeDirectory\\pointclouds\\LACONTEA"
Detection.FileManager.las2pointcloud(source)
plane = Detection.Plane(0,0,1,6.50)
thickness = 0.10
PC = source2pc(source,plane,thickness)
bbin = Detection.FileManager.las2aabb(source)
model = OrthographicProjection.Common.plane2model(plane,thickness,bbin)
