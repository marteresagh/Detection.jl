using Detection
using Visualization
using Common
using FileManager
using Statistics

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/PAVIMENTO"
INPUT_PC = Detection.source2pc(source,2)

# user parameters
par = 0.7
failed = 100
N = 100
k = 60

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,k)

# normals
normals = Common.compute_normals(INPUT_PC.coordinates,threshold,k)
INPUT_PC.normals = normals

# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
@time hyperplanes = Detection.iterate_random_detection(params;debug = true)
hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)
centroid = Common.centroid(INPUT_PC.coordinates)
V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)

GL.VIEW([
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
			# GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],1.0)
		])



GL.VIEW([	
			Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[2]),
			#GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],1.0)
			])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
])


for hyperplane in hyperplanes
	res = Common.residual(hyperplane).([hyperplane.inliers.coordinates[:, i] for i in 1:hyperplane.inliers.n_points])
	max = maximum(res)
	@show max
end
