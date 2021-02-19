using Detection
using Visualization
using Common
using FileManager

# FOLDER = "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS/SEZIONE"
# NAME_PROJ =  "SEZIONE_z=11_8_new"
# NAME_PROJ =  "SEZIONE_z=10_5"

FOLDER = "C:/Users/marte/Documents/Julia_package/package_test/TEST/VECT_1D"
NAME_PROJ =  "PROFILO"; plane = Plane(0, 1, 0, 14.45)
NAME_PROJ =  "PLANIMETRIA"; plane = Plane(0, 0, 1, 6.50)


dirs = Detection.Vect_1D_Dirs(FOLDER, NAME_PROJ)

plane = Plane(0, 1, 0, 14.45)

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

# 3D
GL.VIEW(
    [
    Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs),
    #Visualization.points(Common.apply_matrix(plane.matrix,Common.apply_matrix(Lar.t(-centroid...),PC.coordinates))[1:2,:],GL.COLORS[1],0.2),
    GL.GLGrid(V,EV,GL.COLORS[2],1.0),
#    GL.GLFrame2
    ]
)


L,EL = FileManager.load_segment(joinpath(dirs.RAW,"segment2D.ext"))
fitted_points = FileManager.load_points(joinpath(dirs.RAW,"fitted2D.pnt"))
unfitted_points = FileManager.load_points(joinpath(dirs.RAW,"unfitted2D.pnt"))
GL.VIEW(
    [
    Visualization.points(fitted_points[1:2,:],GL.COLORS[1],0.2),
    Visualization.points(unfitted_points[1:2,:],GL.COLORS[2],0.2),
    GL.GLGrid(L,EL,GL.COLORS[12],1.0),
#    GL.GLFrame2
    ]
)
