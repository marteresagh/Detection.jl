using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"

#######################  REMOVE POINTS
fname = "examples/muriAngolo.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
current_inds = [1:PC2D.n_points...]
k = 5
outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	#GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
  			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12])
		])

par = 0.07
threshold = 2*0.03
failed = 100
N = 100
current_inds = [1:PC2D.n_points...]
visited = copy(outliers)
params = Initializer(PC2D,par,threshold,failed,N,visited,current_inds)


hyperplanes = Detection.iterate_random_detection(params)

presi = setdiff!([1:PC.n_points...],params.current_inds)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])
L,EL = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
  			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,current_inds]'),GL.COLORS[12]),
			GL.GLGrid(L,EL,GL.COLORS[8],1.0),
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,presi]'),GL.COLORS[2]) ,
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,params.visited]'),GL.COLORS[1]),

		])


punti = hcat(params.punti_random_iniziali...)
for i in 10:20
	GL.VIEW(
	    [
		GL.GLPoints(convert(Lar.Points,params.possible_seeds[i]'),GL.COLORS[2]),
		GL.GLPoints(convert(Lar.Points,punti[:,i]'),GL.COLORS[12]),
	    ]
	)
end

hyperplane.points = PC2D
GL.VIEW([
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]),
			GL.GLGrid(L,EL,GL.COLORS[8],1.0),
])


visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])
L,EL = Common.DrawLine(hyperplane,0.0)
GL.VIEW([

			GL.GLGrid(L,EL,GL.COLORS[8],1.0),
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,presi]'),GL.COLORS[2]) ,
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,params.visited]'),GL.COLORS[1]),

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
