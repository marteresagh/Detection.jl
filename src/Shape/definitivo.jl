#### TODO questa funzione va definita nello script per non dare a questo modulo la dipendenza dal pacchetto alpha structures
# function get_boundary_alpha_shape(hyperplane::Hyperplane,plane::Plane)
# 	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
# 	points = hyperplane.inliers.coordinates
# 	V = Common.apply_matrix(plane.matrix,points)[1:2,:]
#
# 	# 2. applica alpha shape con alpha = threshold
# 	filtration = AlphaStructures.alphaFilter(V);
# 	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
#
# 	# 3. estrai bordo
# 	EV_boundary = Common.get_boundary_edges(V,FV)
# 	return Lar.simplifyCells(V,EV_boundary)
# end

## INPUT = V,EV del bordo

function linearization(V::Lar.Points,EV::Lar.Cells)
	graph = graph_edge2edge(V,EV)

	conn_comps = connected_components(graph)

	for comp in conn_comps # indice degli spigoli nella componente


	end
end
