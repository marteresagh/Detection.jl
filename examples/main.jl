using Detection
using Visualization
using Common
using FileManager


# ============== DEBUG
source = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"

folder = "C:/Users/marte/Documents/GEOWEB/TEST"
filename = "TEST_MAIN"
PC = FileManager.las2pointcloud(source)
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
seeds = ""
par = 0.07
failed = 10
N = 10
k = 60
affine_matrix = Lar.t(0,0,6.50)

hyperplanes,params = Detection.pc2vectorize(
							folder,
							filename,
							PC,
	 						par,
							failed,
							N,
							k,
							affine_matrix
							)

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])

presi = setdiff!(collect(1:PC.n_points),params.current_inds)
punti_presi = PointCloud(PC.coordinates[:,presi],PC.rgbs[:,presi])
punti_rimasti = PointCloud(PC.coordinates[:,params.current_inds],PC.rgbs[:,params.current_inds])
V,EV = Common.DrawLines(hyperplanes, 0.0)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[12]),
			#GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])



GL.VIEW([  # GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,params.current_inds]'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,PC_unfitted.coordinates[1:2,:]'),GL.COLORS[12]),
            GL.GLPoints(convert(Lar.Points,PC_fitted.coordinates[1:2,:]'),GL.COLORS[2]),

            GL.GLGrid(V,EV,GL.COLORS[1],1.0),
        ])
