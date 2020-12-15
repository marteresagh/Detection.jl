using Detection
using Visualization
using Common
using AlphaStructures
using FileManager
using Statistics

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/COLONNA"
INPUT_PC = Detection.source2pc(source,0)

# user parameters
par = 0.07
failed = 100
N = 10
k = 30

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,k)

# normals
normals = Common.compute_normals(INPUT_PC.coordinates,threshold,k)
INPUT_PC.normals = normals

# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
@time planes = Detection.iterate_random_detection(params;debug = true)
# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)
centroid = Common.centroid(INPUT_PC.coordinates)
V,FV = Common.DrawPlanes(planes, nothing, 0.0)

GL.VIEW([
			#Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates[:,outliers])'),GL.COLORS[2]) ,
  			GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
		])



GL.VIEW([
			Visualization.mesh_planes(planes,Lar.t(-centroid...))...,
			#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[2]),
			#GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],1.0)
			])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
])


filename = "EV_boundary.txt"
io = open(filename,"w")
for hyperplane in hyperplanes
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	plane = Plane(hyperplane.direction..., Lar.dot(hyperplane.direction,hyperplane.centroid))
	V = Common.apply_matrix(Lar.inv(plane.matrix),points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)

	# 4. salva i segmenti del bordo in 3D
	for ev in EV_boundary
		write(io, "$(V[1,ev[1]]) $(V[2,ev[1]]) $(V[3,ev[1]]) $(V[1,ev[2]]) $(V[2,ev[2]]) $(V[3,ev[2]])\n")
	end

end
close(io)
