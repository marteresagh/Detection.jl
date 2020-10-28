using Detection
using Visualization
using Common
using FileManager


# ============== DEBUG
fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z650.las"
PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
current_inds = [1:PC2D.n_points...]
k = 20
outliers = Common.outliers(PC2D, current_inds, k)
da_tenere = setdiff(current_inds,outliers)

GL.VIEW([  	#GL.GLPoints(convert(Lar.Points,PC2D.coordinates'),GL.COLORS[2]) ,
  			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),
			GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12])
		])

par = 0.07
threshold = 2*0.03
failed = 100
N = 100
current_inds = [1:PC2D.n_points...]
visited = copy(outliers)
params = Initializer(PC2D,par,threshold,failed,N,visited,current_inds)

hyperplanes = Detection.iterate_random_detection(params)

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


GL.VIEW([   GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,params.current_inds]'),GL.COLORS[12]),
            GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,presi]'),GL.COLORS[2]),
            GL.GLGrid(L,EL,GL.COLORS[1],1.0),
        ])

# ============== DEBUG
