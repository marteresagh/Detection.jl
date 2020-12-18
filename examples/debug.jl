using Detection
using Visualization
using Common
using AlphaStructures
using FileManager
using LightGraphs

############################################# ESTRAZIONE BORDO + linearizzazione
function get_boundary_shapes(filename::String, hyperplanes::Array{Hyperplane,1})
	models = Lar.LAR[]
	# io = open(filename,"w")
	for i in 1:length(hyperplanes)

		if i%10 == 0
			Detection.flushprintln("$i planes processed")
		end

		hyperplane = hyperplanes[i]

		# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
		points = hyperplane.inliers.coordinates
		plane = Plane(hyperplane.direction..., Lar.dot(hyperplane.direction,hyperplane.centroid))
		V = Common.apply_matrix(Lar.inv(plane.matrix),points)[1:2,:]

		# 2. applica alpha shape con alpha = threshold
		filtration = AlphaStructures.alphaFilter(V);
		_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

		# 3. estrai bordo
		EV_boundary = Common.get_boundary_edges(V,FV)
		W,EW = Lar.simplifyCells(V,EV_boundary)
		return W,EW
		# 4. linearizzo quando sono sul 2D

		# 5. salvo il modello 3D finale
		# T = Common.points_projection_on_plane(points, hyperplane)
		# W,EW = Lar.simplifyCells(T,EV_boundary)
		# push!(models,(W,EW))
	end

	# return models
end

function get_edges(V::Lar.Points,EV::Lar.Cells)
	# generare grafo


	# intersecare segmenti adiacenti

	# salvare

	return V,EV
end


function linearization(V,EV)
	graph = SimpleGraph(size(V,2))
	for ev in EV
		add_edge!(graph,ev...)
	end

	# estrarre componenti connesse
	conn_comps = connected_components(graph)

	for comp in conn_comps

	end
end


filename = "C:/Users/marte/Documents/GEOWEB/TEST/VECT_2D/EV_boundary_MURI_LOD1.txt"
W,EW = get_boundary_shapes(filename, hyperplanes)

# V,EV = FileManager.load_segment(filename)
# GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],1.0)])

GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EW,GL.COLORS[1],1.0)])

EV_current = linearization(W,EW,conn_comps[1])
GL.VIEW([GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W),EV_current,GL.COLORS[1],1.0)])

graph = SimpleGraph(size(V,2))
for ev in EV_current
	add_edge!(graph,ev...)
end



#  per salvare e leggere STRUTTURE IN UN FILE JLD
#using JLD
#
# for i in 1:length(hyperplanes)
# 	filename = "HYPERPLANES\\hyperplanes$i.jld"
# 	jldopen(filename, "w") do file
# 		write(file, "hyperplane", hyperplanes[1])
# 	end
# end
#
# obj2 = jldopen(filename) do file
#     read(file, "hyperplane")
# end
