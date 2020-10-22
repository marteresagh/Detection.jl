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

par = 0.1
threshold = 2*0.03
failed = 400
N = 50
hyperplanes, currents_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)
line = Detection.get_hyperplane_from_random_init_point(PC2D, [1:PC2D.n_points...], par, threshold)

GL.VIEW([
    mesh_line([line])
])
