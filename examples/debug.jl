using Detection
using Visualization
using Common
using FileManager

fname = "examples/muriAngolo.las"
PC = FileManager.las2pointcloud(fname)

# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )

PC2D = PointCloud(PC.coordinates[1:2,:],PC.rgbs)

par = 0.07
threshold = 2*0.03
failed = 400
N = 50
hyperplanes, current_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

using Statistics
hyperplane = hyperplanes[4]
points = hyperplane.points.coordinates
R = [1:hyperplane.points.n_points...]
res = Common.residual(hyperplane).([points[:,i] for i in R])

mu = Statistics.mean(res)
rho = Statistics.varm(res,mu)

s = (res.-mu).^2

filt = [s[i] < rho for i in 1:length(s)  ]
tokeep = Detection.punti_da_buttare!(points,R,hyperplane)
L,EL = Common.DrawLine(hyperplane,0.0)


GL.VIEW([   GL.GLPoints(convert(Lar.Points,points'),
            GL.COLORS[1]),GL.GLPoints(convert(Lar.Points,points[:,tokeep]'),
            GL.COLORS[2]),GL.GLGrid(L,EL,GL.COLORS[1],1.0)
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
N = 50
hyperplanes, current_inds = Detection.iterate_random_detection(PC, par, threshold, failed, N)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])
