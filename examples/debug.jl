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

par = 0.02
threshold = 2*0.03
failed = 400
N = 50
hyperplanes, current_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

visual = mesh_lines(hyperplanes)
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
