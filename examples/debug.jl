using Detection
using Visualization
using Common
using FileManager
#using Statistics

fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"

#######################  REMOVE POINTS
fname = "examples/muriAngolo.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
current_inds = [1:PC2D.n_points...]
k = 20

outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	#GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),

		])

par = 0.07
threshold = 2*0.03
failed = 10
N = 100

params = Initializer(PC2D,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params)


# # ======================= INPUT generation === Semi-cerchio
# npoints = 2000
# angles = pi*rand(npoints)
#
# V = zeros(2,npoints)
#
# for i in 1:npoints
#     V[1,i] = cos(angles[i])+0.01*rand()
#     V[2,i] = sin(angles[i])+0.01*rand()
# end
#
#
# PC = PointCloud(V,ones(3,npoints))
# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )
#
# par = 0.02
# threshold = 2*0.03
# failed = 400
# N = 100
# params = Initializer(PC,par,threshold,failed,N,10,[])
# hyperplanes, current_inds = Detection.iterate_random_detection(params)
#
# visual = Visualization.mesh_lines(hyperplanes)
# GL.VIEW([visual...])
#
#
#
# fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\prova\\PLANE\\PLANE_vectorized_1D.txt"
#
# V,EV = FileManager.load_segment(fname)
#
# GL.VIEW([
#
# 			GL.GLGrid(V,EV,GL.COLORS[1],1.0),
# 		])
#
