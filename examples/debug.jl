using Detection
using Visualization
using Common
using FileManager
using Statistics


function update_hyperplanes(hyperplanes::Array{Hyperplane,1})
    new_hyperplanes = Hyperplane[]

    for hyperplane in hyperplanes
        points = hyperplane.points.coordinates
        R = [1:hyperplane.points.n_points...]

        res = Common.residual(hyperplane).([points[:,i] for i in R])
        mu = Statistics.mean(res)
        rho = Statistics.std(res)

        todel = [mu - rho < res[i] < mu + rho for i in 1:length(res)  ]

        # Detection.punti_da_tenere!(points,R,hyperplane)
        listPoint = points[:,R[todel]]
        direction, centroid = Common.LinearFit(listPoint)
        hyperplane_update = Hyperplane(PointCloud(listPoint),direction,centroid)
        push!(new_hyperplanes,hyperplane_update)
    end
    return new_hyperplanes
end


fname = "examples/wall.las"
fname = "examples/muriAngolo.las"
fname = "examples/area.las"
PC = FileManager.las2pointcloud(fname)

# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )

PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)

par = 0.07
threshold = 2*0.03
failed = 200
N = 100
hyperplanes, current_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

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

new_hyperplanes = update_hyperplanes(hyperplanes)

L,EL = Common.DrawLines(hyperplanes,0.0)
T,ET = Common.DrawLines(new_hyperplanes,0.0)

GL.VIEW([   GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[1]),
            #GL.GLPoints(convert(Lar.Points,points[:,R[todel]]'),GL.COLORS[2]),
            GL.GLGrid(L,EL,GL.COLORS[2],0.8),
            GL.GLGrid(T,ET,GL.COLORS[12],1.0)
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
