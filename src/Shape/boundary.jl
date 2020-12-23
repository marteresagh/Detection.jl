# using AlphaStructures
#
# function boundary_shapes(hyperplanes::Array{Hyperplane,1}, threshold::Float64)::Lar.LAR
# 	out = Array{Lar.Struct,1}()
# 	for i in 1:length(hyperplanes)
#
# 		if i%10 == 0
# 			Detection.flushprintln("$i planes processed")
# 		end
#
# 		hyperplane = hyperplanes[i]
#
# 		# 1. applica matrice di rotazione agli inliers ed estrai i punti 2D
# 		points = hyperplane.inliers.coordinates
# 		plane = Plane(hyperplane.direction, hyperplane.centroid)
# 		V = Common.apply_matrix(plane.matrix,points)[1:2,:]
#
# 		# 2. applica alpha shape con alpha = threshold
# 		filtration = AlphaStructures.alphaFilter(V);
# 		_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
#
# 		# 3. estrai bordo
# 		EV_boundary = Common.get_boundary_edges(V,FV)
# 		W,EW = Lar.simplifyCells(V,EV_boundary)
# 		models = process(W,EW)
#
# 		for model in models
# 			vertices = Common.apply_matrix(Lar.inv(plane.matrix), vcat(model[1],zeros(size(model[1],2))'))
# 			out = push!(out, Lar.Struct([(vertices, model[2])]))
# 		end
#
# 	end
# 	out = Lar.Struct(out)
# 	V,EV = Lar.struct2lar(out)
#
# 	# 5. salvo il modello come??
#
# 	return V,EV
# end

function get_linerized_models(input_model::Lar.LAR)::Array{Lar.LAR,1}
	V,EV = input_model
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
