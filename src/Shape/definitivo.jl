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
	graph = Common.model2graph_edge2edge(V,EV)
	conn_comps = connected_components(graph)

	for comp in conn_comps # indice degli spigoli nella componente
		subgraph = induced_subgraph(graph, comp)
		clus = clusters(V,EV, deepcopy(subgraph), 0.1)
		#dict_clusters, graph_cluadj = graph_adjacency_clusters(V, EV, subgraph, clusters)
		pol = polyline(V, EV, clus)
		out = push!(out, Lar.Struct([pol]))
	end

	out = Lar.Struct(out)
	return Lar.struct2lar(out)
end


function graph_adjacency_clusters(V, EV, clusters)
	graph_verts = SimpleGraph(size(V,2))
	for ev in EV
		add_edge!(graph_verts, ev[1],ev[2])
	end

	n_cluss = length(clusters)
	dict_clusters = DataStructures.OrderedDict([i => clusters[i] for i in 1:n_cluss]...)
	graph = SimpleGraph(n_cluss)

	for (k,v) in dict_clusters
		M_1 = Common.K(EV[v])
		∂_1 = M_1'
		S1 = sum(∂_1,dims=2)
		outers = [k for k=1:length(S1) if S1[k]==1]
		N = union([neighbors(graph_verts, outer) for outer in outers]...)

		for n in N
			for (kn,vn) in dict_clusters
				if in(n,union(EV[vn]...))
					if k!=kn
						add_edge!(graph,k,kn)
					end
				end
			end
		end
	end
	return dict_clusters, graph
end


function valid(cluster)
	return length(cluster)>5
end

# TODO costruire grafo adiacenza cluster
function clusters(V, EV, subgraph, par)
	grph, vmap = subgraph

	linear_clusters = Array{Int64,1}[]

	ITER = 0
	while ITER < 100

		try
			R, linear_cluster = clustering_edge(V, EV, grph, vmap, par)
			if valid(linear_cluster)
				push!(linear_clusters, linear_cluster)
				a = rem_vertices!(grph, R, keep_order=true);
				vmap = vmap[a]
			end
		catch y
		end

		ITER = ITER + 1
	end

	conn_comps = connected_components(grph)
	for comp in conn_comps
		if length(comp) == 1
			edge = vmap[comp[1]]
			dist = Lar.norm(V[:,EV[edge][1]]-V[:,EV[edge][2]])
			if dist > 1.
				push!(linear_clusters, [edge])
			end
		end
	end

	return linear_clusters
end

function clustering_edge(V, EV, grph, vmap, par)
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
					if Common.residual(line)(point) < par
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
				if candidate && Common.angle_between_directions(edge_dir,direction) < pi/5
					push!(R,neighbor)
					push!(tmp,neighbor)
				end
			end
		end

		seeds = tmp
	end
	return R,vmap[R]
end

function polyline(V, EV, clusters)
	#grafo
	dict_clusters, graph_clus = graph_adjacency_clusters(V, EV, clusters)

	out = Array{Lar.Struct,1}()
	for cluster in clusters
		M_1 = Common.K(EV[cluster])
		∂_1 = M_1'
		S1 = sum(∂_1,dims=2)
		outer = [k for k=1:length(S1) if S1[k]==1]
		inds = union(EV[cluster]...)
		inliers = setdiff(inds,outer)
		direction, centroid = Common.Fit_Line(V[:,inliers])
		line = Hyperplane(PointCloud(V[:,inds]),direction,centroid)
		L,EL = Common.DrawLines(line)
		out = push!(out, Lar.Struct([(L,EL)]))
	end

	out = Lar.Struct(out)
	return Lar.struct2lar(out)
end
