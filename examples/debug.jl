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
function get_boundary_shapes(filename::String, hyperplanes, threshold)
	# io = open(filename,"w")
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end

		hyperplane = hyperplanes[i]

		# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
		points = hyperplane.inliers.coordinates
		plane = Plane(hyperplane.direction..., Lar.dot(hyperplane.direction,hyperplane.centroid))
		V = Common.apply_matrix(plane.matrix,points)[1:2,:]

		# 2. applica alpha shape con alpha = threshold
		filtration = AlphaStructures.alphaFilter(V);
		_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

		# 3. estrai bordo
		EV_boundary = Common.get_boundary_edges(V,FV)
		W,EW = Lar.simplifyCells(V,EV_boundary)
		models = process(W,EW)

		for model in models
			vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
			out = push!(out, Lar.Struct([(vertices, model[2])]))
		end

		# 4. linearizzo quando sono sul 2D

		# 5. salvo il modello 3D finale
		# T = Common.points_projection_on_plane(points, hyperplane)
		# W,EW = Lar.simplifyCells(T,EV_boundary)
		# push!(models,(W,EW))
	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)
	return V,EV
	# return models
end

function process(V,EV)
	models = Lar.LAR[]
	graph = SimpleGraph(size(V,2))
	for ev in EV
		add_edge!(graph,ev...)
	end

	# estrarre componenti connesse
	conn_comps = connected_components(graph)
	for comp in conn_comps
		model = linearization(graph,V,comp)
		if !isnothing(model)
			push!(models,model)
		end
	end
	return models
end

function linearization(graph,V,comp)
	clusters = Hyperplane[]
	comp_current = comp
	while length(comp_current) > 1
		cluster_hyperplane,index_visited = get_line(graph,V,comp_current)
		comp_current = setdiff(comp_current,index_visited)
		push!(clusters,cluster_hyperplane)
	end

	if !isempty(clusters)
		return Common.DrawLines(clusters,0.0)
	end
	return nothing
end

function get_line(graph,V,comp_current)
	first = rand(comp_current)
	N = neighborhood(graph,first,1)
	points = V[:,N]
	direction, centroid = Common.LinearFit(points)
	hyperplane = Hyperplane(PointCloud(points), direction, centroid)
	seeds = N
	index_visited = N
	while !isempty(seeds)
		for i in seeds
			N = neighborhood(graph,i,1)
			setdiff!(N,index_visited)
			tmp = Int[]
			for near in N
				if Common.residual(hyperplane)(V[:,near]) < 0.1
					points = hcat(points,V[:,near])
					push!(tmp, near)
					push!(index_visited,near)
				end
				direction, centroid = Common.LinearFit(points)
			   	hyperplane = Hyperplane(PointCloud(points), direction, centroid)
			end
			seeds = tmp
		end
	end
	return hyperplane, index_visited
end

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
