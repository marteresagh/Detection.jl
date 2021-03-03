### nuovo algoritmo per linearizzare gli spigoli
function test(models)
    V,EV = models[1]
    graph = Common.model2graph_edge2edge(V,EV)
	all_clus = []
    conn_comps = LightGraphs.connected_components(graph)
    for comp in conn_comps[1:end]
        subgraph = LightGraphs.induced_subgraph(graph, comp)
        clus = find_clusters(V,EV, deepcopy(subgraph))
		push!(all_clus,clus)
	end

	return all_clus
end

function find_clusters(P::Lar.Points, EV::Lar.Cells, subgraph)

	function direction(V,EV,e)
		inds = EV[vmap[e]]
		dir = V[:, inds[1]]-V[:, inds[2]]
		dir /=Lar.norm(dir)
		return dir
	end

	plane = Plane(P)
	V = Common.apply_matrix(plane.matrix,P)[1:2,:]

	grph, vmap = subgraph
	seen = zeros(Bool,LightGraphs.nv(grph))
	current_ind = [1:LightGraphs.nv(grph)...]
	clusters_found = Array{Int64,1}[]

	while !isempty(current_ind)
		e1 = rand(current_ind)
		seen[e1] = true
		cluster = [vmap[e1]]
		visited = [e1]
		S = [e1]

		dir_ref = direction(V,EV,e1)

		while !isempty(S)

			e = pop!(S)
			neighbors = LightGraphs.neighborhood(grph,e,1)
			for neighbor in neighbors
				if !seen[neighbor]
					dir = direction(V,EV,neighbor)
					if Common.angle_between_directions(dir_ref,dir)<=pi/6
						push!(cluster,vmap[neighbor])
						push!(S,neighbor)
						seen[neighbor]=true
						push!(visited,neighbor)
					end
				end
			end
			dir_ref,_ = Common.LinearFit(V[:,union(EV[cluster]...)])
		end

		push!(clusters_found, cluster)
		setdiff!(current_ind,visited)

	end

	return clusters_found
end
