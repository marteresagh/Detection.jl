using Detection
using Visualization
using Common
using AlphaStructures
using FileManager
using LightGraphs
using JLD
################################################################################ 3D
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

# user parameters
par = 0.04
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
@time hyperplanes = Detection.iterate_random_detection(params;debug = true)

############################################# ESTRAZIONE BORDO + linearizzazione

filename = "C:/Users/marte/Documents/GEOWEB/TEST/VECT_2D/EV_boundary_MURI_LOD1.txt"
#
# filename = "HYPERPLANES\\hyperplanes1.jld"
# hyperplane = jldopen(filename) do file
#     read(file, "hyperplane")
# end
#
# hyperplanes = [hyperplane]

V,EV = get_boundary_shapes(filename, hyperplanes, threshold)

GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],1.0)])

graph, comp_current = process(W,EW)
line,index = get_line(graph,W,comp_current)

V,EV = Common.DrawLine(line,0.0)

V,EV = linearization(graph,W,comp_current)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),W)'),GL.COLORS[12]),
  			GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),V),EV,GL.COLORS[1],1.0)
		])

GL.VIEW([GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(v_comp)...),v_comp)'),GL.COLORS[1])])

############################################################## Linearizazione 2D
# non va bene la vettorizzazione che uso per il 2D
# provo con una specie di ransac
INPUT2D_PC = PointCloud(W[:,comp_current], rand(3,size(W[:,comp_current],2)))

# user - parameters
par = 0.07
failed = 100
N = 10
k = 10

# threshold estimation
thres = Common.estimate_threshold(INPUT2D_PC,k)

# outliers
outliers = Int[]#Common.outliers(INPUT2D_PC, collect(1:INPUT2D_PC.n_points), k)

# process
params = Initializer(INPUT2D_PC,par,thres,failed,N,k,outliers)
lines = Detection.iterate_random_detection(params,debug = true)
# line,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(lines,0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT2D_PC.coordinates)...),INPUT2D_PC.coordinates)'),GL.COLORS[12]),
  			GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(INPUT2D_PC.coordinates)...),V),EV,GL.COLORS[1],1.0)
		])

#################################################################################################

# V,EV = FileManager.load_segment(filename)
# GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],1.0)])



#  per salvare e leggere STRUTTURE IN UN FILE JLD
using JLD

# for i in 1:length(hyperplanes)
# 	filename = "HYPERPLANES\\hyperplanes$i.jld"
# 	jldopen(filename, "w") do file
# 		write(file, "hyperplane", hyperplanes[1])
# 	end
# end
