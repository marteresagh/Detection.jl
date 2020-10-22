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
GL.VIEW([Visualization.points_color_from_rgb(PC2D.coordinates,PC2D.rgbs,0.4),visual...])
