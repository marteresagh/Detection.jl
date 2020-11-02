using Detection
using Visualization
using Common
using FileManager


# ============== DEBUG
source = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z650.las"

folder = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\prova"
filename = "script"
PC = FileManager.las2pointcloud(source)
par = 0.07
threshold = 2*0.03
failed = 10
N = 100
k = 10
affine_matrix = Lar.t(0,0,6.50)

# direction,centroid = Common.LinearFit(PC.coordinates)

hyperplanes,params = Detection.detection_and_saves(
							folder,
							filename,
							source,
	 						par,
							threshold,
							failed,
							N,
							k,
							affine_matrix
							)

# julia detection.jl "C:\Users\marte\Documents\GEOWEB\FilePotree\orthoCONTEA\Sezione_z650.las" -p "PLANE" -o "C:\Users\marte\Documents\GEOWEB\FilePotree\TEST_LINES\prova" --failed 10 --par 0.02 --thr 0.06 --plane "0 0 1 6.50"

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])

presi = setdiff!([1:PC.n_points...],params.current_inds)
punti_presi = PointCloud(PC.coordinates[:,presi],PC.rgbs[:,presi])
punti_rimasti = PointCloud(PC.coordinates[:,params.current_inds],PC.rgbs[:,params.current_inds])
V,EV = Common.DrawLines(hyperplanes, 0.0)


GL.VIEW(
    [
    GL.GLGrid(V,EV,GL.COLORS[1],1.0)
    ]
)


GL.VIEW([  # GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,params.current_inds]'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,PC_unfitted.coordinates[1:2,:]'),GL.COLORS[12]),
            GL.GLPoints(convert(Lar.Points,PC_fitted.coordinates[1:2,:]'),GL.COLORS[2]),

            GL.GLGrid(V,EV,GL.COLORS[1],1.0),
        ])

# ============== DEBUG
# julia detection.jl "C:\Users\marte\Documents\GEOWEB\FilePotree\TEST_LINES\Sezione_z39_10cm.las" -p "ALTRASEZ" -o "C:\Users\marte\Documents\GEOWEB\FilePotree\TEST_LINES\prova" --failed 10 --par 0.02 --thr 0.02 --plane "0 0 1 6.50"
