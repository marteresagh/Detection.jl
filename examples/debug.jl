using Detection
using Visualization
using Common
using AlphaStructures
using FileManager
using AlphaStructures

function get_boundary_alpha_shape(hyperplane::Hyperplane,plane::Plane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return V,EV_boundary
end

function boundary_shapes(hyperplanes::Array{Hyperplane,1}, threshold::Float64)::Lar.LAR
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		V,EV_boundary = get_boundary_alpha_shape(hyperplane,plane)

		input_model = Lar.simplifyCells(V,EV_boundary)
		models = Detection.get_linerized_models(input_model)

		for model in models
			vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
			out = push!(out, Lar.Struct([(vertices, model[2])]))
		end

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)

	# 5. salvo il modello come??

	return V,EV
end

################################################################################ 3D
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

# user parameters
par = 0.05
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
hyperplane = hyperplanes[4]
points = hyperplane.inliers.coordinates
plane = Plane(hyperplane.direction, hyperplane.centroid)
V = Common.apply_matrix(plane.matrix,points)[1:2,:]

# 2. applica alpha shape con alpha = threshold
filtration = AlphaStructures.alphaFilter(V);
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

# 3. estrai bordo
EV = Common.get_boundary_edges(V,FV)

V,EV = boundary_shapes(hyperplanes, threshold)

GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV_boundary,GL.COLORS[1],1.0)])


####################################################################
# V,EV = FileManager.load_segment(filename)
# GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],1.0)])

#  per salvare e leggere STRUTTURE IN UN FILE JLD
# for i in 1:length(hyperplanes)
# 	filename = "HYPERPLANES\\hyperplanes$i.jld"
# 	jldopen(filename, "w") do file
# 		write(file, "hyperplane", hyperplanes[1])
# 	end
# end
