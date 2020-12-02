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

failed = 100
N = 100
INPUT_PC = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
k = 20

# threshold estimation
threshold = Detection.estimate_threshold(INPUT_PC,k)

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



################# test of validity
using Plots
new_hyp = Hyperplane[]
for i in 1:length(hyperplanes)
	hyperplane = hyperplanes[i]
	pc_on_hyperplane = hyperplane.inliers
	_, local_density = Common.relative_density_points(pc_on_hyperplane.coordinates, collect(1:pc_on_hyperplane.n_points), Int(floor(pc_on_hyperplane.n_points/2)))
	dist = map(x->1/x,local_density)
	mu = Statistics.mean(dist)

	if mu < 1.5
		push!(new_hyp,hyperplane)
	end
	E,_ = Common.DrawLine(hyperplane, 0.0)
	dist = Lar.norm(E[:,1]-E[:,2])
	rho = pc_on_hyperplane.n_points/dist
	@show rho
	# questo è un ottimo metodo
	# res = Common.residual(hyperplane).([hyperplane.inliers.coordinates[:,i] for i in 1:hyperplane.inliers.n_points])
	# mu = Statistics.mean(res)# prova moda
	# rho = Statistics.std(res)
	# if mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005
	# 	push!(new_hyp,hyperplane)
	# end
end

GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(new_hyp)...])

for i in 1:length(hyperplanes)
	@show i
	GL.VIEW([	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
				Visualization.mesh_lines([hyperplanes[i]])...])
end
