using Detection
using Visualization
using Common
using FileManager

FOLDER = "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS/SEZIONE"
NAME_PROJ = "SEZIONE_z=11_8"  #"SEZIONE_z=10_5"
dirs = Detection.VectDirs(FOLDER, NAME_PROJ)

plane = Plane(0, 0, 1, 10.5)

V,EV = FileManager.load_segment(joinpath(dirs.RAW,"segment3D.ext"))
PC = FileManager.las2pointcloud(joinpath(dirs.FULL,"slice.las"))


centroid = Common.centroid(V)
GL.VIEW(
    [
    Visualization.points_color_from_rgb(Common.apply_matrix(plane.matrix,Common.apply_matrix(Lar.t(-centroid...),PC.coordinates))[1:2,:],PC.rgbs),
    #Visualization.points(Common.apply_matrix(plane.matrix,Common.apply_matrix(Lar.t(-centroid...),PC.coordinates))[1:2,:],GL.COLORS[1],0.2),
    GL.GLGrid(Common.apply_matrix(plane.matrix,Common.apply_matrix(Lar.t(-centroid...),V))[1:2,:],EV,GL.COLORS[2],1.0),
#    GL.GLFrame2
    ]
)
