using Common
using FileManager
using Detection
using Visualization
using AlphaStructures

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

# user parameters
par = 0.05
failed = 100
N = 30
k = 30

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,k)

# normals
normals = Common.compute_normals(INPUT_PC.coordinates, threshold, k)
INPUT_PC.normals = normals

# seeds indices
masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_COLOMBELLA.txt"
given_seeds = FileManager.load_points(masterseeds)
seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

seeds = Int64[]
# outliers
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)

# 2. Detection
hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)
#hyperplane, cluster, all_visited_verts = Detection.get_hyperplane(params; given_seed = seeds[1])
centroid = Common.centroid(INPUT_PC.coordinates)
V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)

GL.VIEW([
	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates[:,outliers])'),GL.COLORS[2]) ,
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])

############################## seconda parte
# Alpha shape model

function save_alpha_shape_model(hyperplanes::Array{Hyperplane,1}, threshold::Float64, name_proj::String)
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		Detection.flushprintln("$i planes processed")

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		model = get_boundary_alpha_shape(hyperplane,plane)

		vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
		out = push!(out, Lar.Struct([(vertices, model[2])]))

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)
	FileManager.save_points_txt(name_proj*"_points.txt", V)
	FileManager.save_cells_txt(name_proj*"_edges.txt", EV)
	return V,EV
end

function get_boundary_alpha_shape(hyperplane::Hyperplane, plane::Plane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	filtration = AlphaStructures.alphaFilter(V);
	threshold = Common.estimate_threshold(hyperplane.inliers,10)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
end

#main
function boundary_shapes(hyperplanes::Array{Hyperplane,1})::Lar.LAR
	out = Array{Lar.Struct,1}()
	for i in 1:length(hyperplanes)

		Detection.flushprintln("$i planes processed")

		hyperplane = hyperplanes[i]
		plane = Plane(hyperplane.direction, hyperplane.centroid)

		input_model = get_boundary_alpha_shape(hyperplane,plane)

		model = Detection.linearization(input_model...) #models = Detection.get_linerized_models(input_model)

		vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
		out = push!(out, Lar.Struct([(vertices, model[2])]))

	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)

	# 5. salvo il modello come??

	return V,EV
end


W,EW = save_alpha_shape_model(hyperplanes, threshold, "CHIESA_COLOMBELLA_ashapes")

W,EW = boundary_shapes(hyperplanes)

GL.VIEW([
		#Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
		GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.)
])


for i in 1:length(hyperplanes)
	FileManager.save_hyperplane("HYPERPLANE/CHIESA_COLOMBELLA_$i.txt", hyperplanes[i])
end
