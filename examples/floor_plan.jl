using Detection
using Visualization
using Common
using FileManager
using Statistics

fname = "examples/las/wall.las"
fname = "examples/las/polyline.las"
fname = "examples/las/full.las"
fname = "examples/las/square.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/sezione_AMPHI_z39_5cm.las"

PC = FileManager.las2pointcloud(fname)
par = 0.07
threshold = 0.06
failed = 100
N = 10
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
k = 20
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params,debug = true)

# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])


GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(hyperplanes)...])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])


new_hyp = Hyperplane[]
for i in 1:length(hyperplanes)
	hyperplane = hyperplanes[i]
	res = Common.residual(hyperplane).([hyperplane.inliers.coordinates[:,i] for i in 1:hyperplane.inliers.n_points])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	if mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005
		push!(new_hyp,hyperplane)
	end
end

GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(new_hyp)...])
