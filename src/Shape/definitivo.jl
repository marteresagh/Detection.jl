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
	out = Array{Lar.Struct,1}()
	graph = graph_edge2edge(V,EV)
	conn_comps = connected_components(graph)

	for comp in conn_comps # indice degli spigoli nella componente
		subgraph = induced_subgraph(graph, comp)
		clusters = clusters(V,EV, subgraph)
		pol = polyline(V, EV, subgraph, clusters) #TODO
		out = push!(out, Lar.Struct([pol]))
	end

	out = Lar.Struct(out)
	return Lar.struct2lar(out)
end

# TODO da finire la riduzione del grafo 
function clusters(V,EV,subgraph)
	grph, vmap = subgraph
	clusters = Array{Int64,1}[]

	ITER = 0
	while ITER < 100
		cluster = clustering_edge(V, EV, grph, vmap)
		if valid(cluster)
			push!(clusters,cluster)
			for v in cluster
				rem_vertex!(grph, findall(x-> x == v,vmap))
			end
		end
		ITER = ITER + 1
	end

	return clusters
end

function clustering_edge(V, EV, grph, vmap)
	e1 = rand(1:nv(grph))
	e2 = rand(setdiff(neighborhood(grph,e1,3),e1))

	R = union(e1,e2)
	init = vmap[R]
	seeds = copy(R)
	visited = copy(R)
	indx = union(EV[vmap[R]]...)
	inliers = V[:,indx]
	direction,centroid = Common.LinearFit(inliers)
	line = Hyperplane(PointCloud(inliers), direction, centroid)

	while !isempty(seeds)
		tmp = Int64[]
		for edge in seeds
			neighbors = setdiff(neighborhood(grph,edge,1),visited)
			union!(visited, neighbors)
			for neighbor in neighbors
				candidate = false
				inds = EV[vmap[neighbor]]
				for ind in inds
					point = V[:,ind]
					if Common.residual(line)(point) < 0.1
						inliers = hcat(inliers, point)
						direction,centroid = Common.LinearFit(inliers)
						line.inliers = PointCloud(inliers)
						line.direction = direction
						line.centroid = centroid
						candidate = true
					end
				end
				extrema = EV[vmap[neighbor]]
				edge_dir = V[:,extrema[1]]-V[:,extrema[2]]
				if candidate && Common.angle_between_vectors(edge_dir,direction) < pi/5
					push!(R,neighbor)
					push!(tmp,neighbor)
				end
			end
		end

		seeds = tmp
	end
	return init,vmap[R]
end

function polyline(V, EV, subgraph, clusters)

end
