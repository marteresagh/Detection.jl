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

# ===============

visual = Visualization.mesh_lines(planes)

GL.VIEW([visual...])


L,EL = Common.DrawLines(hyperplanes,0.0)


GL.VIEW([   GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[1]),
            #GL.GLPoints(convert(Lar.Points,points[:,R[todel]]'),GL.COLORS[2]),
            GL.GLGrid(L,EL,GL.COLORS[2],0.8),
        ])

visual = Visualization.mesh_lines([hyperplane])
GL.VIEW([visual...])

visual = Visualization.mesh_lines([Hyperplane(PointCloud(points[:,tokeep]),hyperplane.direction,hyperplane.centroid)])
GL.VIEW([visual...])










# ======================= INPUT generation === Semi-cerchio
npoints = 2000
angles = pi*rand(npoints)

V = zeros(2,npoints)

for i in 1:npoints
    V[1,i] = cos(angles[i])+0.01*rand()
    V[2,i] = sin(angles[i])+0.01*rand()
end


PC = PointCloud(V,ones(3,npoints))
GL.VIEW(
    [
    Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
    ]
)

par = 0.02
threshold = 2*0.03
failed = 400
N = 100
hyperplanes, current_inds = Detection.iterate_random_detection(PC, par, threshold, failed, N)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

#######################  REMOVE POINTS
using NearestNeighbors
function remove_isolated_points(PC, current_inds, k)
	points = PC.coordinates
	kdtree = NearestNeighbors.KDTree(points[:,current_inds])
	idxs, dists = NearestNeighbors.knn(kdtree, points, k, true)

	density = Float64[]
	for i in 1:length(current_inds)
		rho = sum(dists[i])/k
		push!(density,1/rho)
	end
	AVGRelDensity = Float64[]
	for i in 1:length(current_inds)
		rel = density[i]/((1/k)*sum(density[idxs[i]]))
		push!(AVGRelDensity,rel)
	end
	return density,AVGRelDensity
end


fname = "examples/muriAngolo.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
# points = hcat(rand(2,1000),[2.0,2.0])
# PC = PointCloud(points)
current_inds = [1:PC2D.n_points...]
density,AVGRelDensity = remove_isolated_points(PC2D,current_inds,20)


current_inds = [1:PC2D.n_points...]
density,AVGRelDensity = remove_isolated_points(PC2D,current_inds,20)
mu = Statistics.mean(AVGRelDensity)
rho = Statistics.std(AVGRelDensity)
outliers = [AVGRelDensity[i]<mu-rho for i in current_inds ]
da_rimuovere = current_inds[outliers]
da_tenere = setdiff(current_inds,da_rimuovere)
PC = PointCloud(PC2D.coordinates[:,da_tenere],PC2D.rgbs)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
  			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_rimuovere]'),GL.COLORS[1]),
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12])
			 ])




par = 0.02
threshold = 2*0.03
failed = 500
N = 100

hyperplanes, current_inds,visited = Detection.iterate_random_detection(PC, par, threshold, failed, N)
visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

GL.VIEW(
    [
    Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
    ]
)
