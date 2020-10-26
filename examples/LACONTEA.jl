using Detection
using Visualization
using Common
using FileManager

fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z650.las"
PC = FileManager.las2pointcloud(fname)

# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )

PC2D = PointCloud(PC.coordinates[1:2,:],PC.rgbs)

par = 0.07
threshold = 2*0.03
failed = 500
N = 100
hyperplanes, current_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])


new_hyperplanes = update_hyperplanes(hyperplanes)

L,EL = Common.DrawLines(hyperplanes,0.0)
T,ET = Common.DrawLines(new_hyperplanes,0.0)

GL.VIEW([   #GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[1]),
            #GL.GLPoints(convert(Lar.Points,points[:,R[todel]]'),GL.COLORS[2]),
            GL.GLGrid(L,EL,GL.COLORS[1],1.0),
            #GL.GLGrid(T,ET,GL.COLORS[12],1.0)
        ])


presi = setdiff!([1:PC.n_points...],current_inds)
punti_presi= PointCloud(PC.coordinates[:,presi],PC.rgbs[:,presi])


punti_rimasti = PointCloud(PC.coordinates[:,current_inds],PC.rgbs[:,current_inds])

FileManager.save_pointcloud(punti_rimasti, "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z650_puntirimasti.las")

V,EV = Common.DrawLines(hyperplanes, 0.0)

GL.VIEW(
    [
    GL.GLGrid(V,EV,GL.COLORS[1],1.0)
    ]
)

FileManager.save_lines_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\segmenti_z650.txt", hyperplanes, 6.50)
FileManager.save_points_rgbs_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z650_puntipresi.txt", punti_presi)
FileManager.save_points_rgbs_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z650_puntirimasti.txt", punti_rimasti)

#############################################################################  seconda sezione

fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z250.las"
PC = FileManager.las2pointcloud(fname)

# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )

PC2D = PointCloud(PC.coordinates[1:2,:],PC.rgbs)

par = 0.09
threshold = 2*0.03
failed = 500
N = 200
hyperplanes, current_inds = Detection.iterate_random_detection(PC2D, par, threshold, failed, N)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

L,EL = Common.DrawLines(hyperplanes,0.0)

GL.VIEW([   #GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[1]),
            #GL.GLPoints(convert(Lar.Points,points[:,R[todel]]'),GL.COLORS[2]),
            GL.GLGrid(L,EL,GL.COLORS[1],1.0),
        ])



presi = setdiff!([1:PC.n_points...],current_inds)
punti_presi= PointCloud(PC.coordinates[:,presi],PC.rgbs[:,presi])
punti_rimasti = PointCloud(PC.coordinates[:,current_inds],PC.rgbs[:,current_inds])

FileManager.save_pointcloud(punti_presi, "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z250_puntipresi.las")
FileManager.save_pointcloud(punti_rimasti, "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z250_puntirimasti.las")

V,EV = Common.DrawLines(hyperplanes, 0.0)

GL.VIEW(
    [
    GL.GLGrid(V,EV,GL.COLORS[1],1.0)
    ]
)

FileManager.save_lines_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\segmenti_z250.txt", hyperplanes, 2.50)
FileManager.save_points_rgbs_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z250_puntipresi.txt", punti_presi)
FileManager.save_points_rgbs_txt("C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\risultati\\sezione_z250_puntirimasti.txt", punti_rimasti)
