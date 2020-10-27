using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"
PC = FileManager.las2pointcloud(fname)

PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

par = 0.07
threshold = 2*0.03
failed = 200
N = 100
hyperplanes, current_inds, visited = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

presi = setdiff!([1:PC.n_points...],current_inds)

GL.VIEW([
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,current_inds]'),GL.COLORS[2]),
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,visited]'),GL.COLORS[1]),
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,presi]'),GL.COLORS[12]),
        ])

GL.VIEW([Visualization.mesh_lines(hyperplanes)...])

# ===============
hyperplane = hyperplanes[5]
points = hyperplane.points.coordinates
Statistics.cor(points[1,:],points[2,:])
R = [1:hyperplane.points.n_points...]
res = Common.residual(hyperplane).([points[:,i] for i in R])
max(res...)
min(res...)
mu = Statistics.mean(res)
rho = Statistics.stdm(res,0.0)

L,EL = Common.DrawLine(hyperplane,0.0)

GL.VIEW([   GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[1]),
            #GL.GLPoints(convert(Lar.Points,points[:,R[todel]]'),GL.COLORS[2]),

            GL.GLGrid(L,EL,GL.COLORS[12],1.0)
        ])


#######################  REMOVE POINTS
fname = "examples/muriAngolo.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
current_inds = [1:PC2D.n_points...]
k = 10
outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	#GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
  			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12])
		])


par = 0.07
threshold = 2*0.05
failed = 2000
N = 100

hyperplanes, current_inds, visited = Detection.iterate_random_detection(PC2D, par, threshold, failed, N, outliers)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

GL.VIEW(
    [
    Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
    ]
)

presi = setdiff!([1:PC.n_points...],current_inds)

GL.VIEW([
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,current_inds]'),GL.COLORS[2]),
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,visited]'),GL.COLORS[1]),
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,presi]'),GL.COLORS[12]),
        ])



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
# hyperplanes, current_inds = Detection.iterate_random_detection(PC, par, threshold, failed, N)
#
# visual = Visualization.mesh_lines(hyperplanes)
# GL.VIEW([visual...])
