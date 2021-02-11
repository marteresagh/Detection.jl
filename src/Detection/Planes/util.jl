function get_boundary_alpha_shape(hyperplane::Hyperplane)
	# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
	points = hyperplane.inliers.coordinates
	plane = Plane(hyperplane.direction,hyperplane.centroid)
	V = Common.apply_matrix(plane.matrix,points)[1:2,:]

	# 2. applica alpha shape con alpha = threshold
	DT = Common.delaunay_triangulation(V)
	filtration = AlphaStructures.alphaFilter(V,DT);
	threshold = Common.estimate_threshold(hyperplane.inliers,5)
	_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

	# 3. estrai bordo
	EV_boundary = Common.get_boundary_edges(V,FV)
	return Lar.simplifyCells(V,EV_boundary)
end


function save_boundary_shape(folder::String,hyperplane::Hyperplane)
	V,EV = get_boundary_alpha_shape(hyperplane)
	# devo renderlo diretto e cercare i cicli

	io = open(joinpath(folder,"boundary_edges.txt"),"w")
	g = model2graph(V,EV)
	conn_comps = connected_components(g)
	for comp in conn_comps
		subgraph,vmap = induced_subgraph(g, comp)
		path = dfs_tree(subgraph, 1)
		edges = topological_sort_by_dfs(path)
		inds = vmap[edges]
		for ind in inds
			write(io,"$ind ")
		end
		write(io,"\n")
	end
	close(io)
	# embed V,EV in 3D space
	plane = Plane(hyperplane.direction, hyperplane.centroid)
	vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(V,zeros(size(V,2))'))
	FileManager.save_points_txt(joinpath(folder,"boundary_points.txt"), vertices)

	#FileManager.save_cells_txt(joinpath(folder,"boundary_edges.txt"), EV)
end
