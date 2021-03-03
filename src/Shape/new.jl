
function test(models)
    V,EV = models[1]
    graph = Common.model2graph_edge2edge(V,EV)
	all_clus = []
    conn_comps = LightGraphs.connected_components(graph)
    for comp in conn_comps[2:end]
        subgraph = LightGraphs.induced_subgraph(graph, comp)
        clus = find_clusters(V,EV, deepcopy(subgraph))
		push!(all_clus,clus)
		break
	end

	return all_clus
end

# mi serve un dizionario di cluster ("1" => [spigoli], "2" => [spigoli],... )
function find_clusters(V::Lar.Points, EV::Lar.Cells, subgraph)

	function direction(e)
		inds = EV[vmap[e]]
		dir = V[:, inds[1]]-V[:, inds[2]]
		dir /=Lar.norm(dir)
		return dir
	end

	grph, vmap = subgraph
	seen = zeros(Bool,LightGraphs.nv(grph))
	current_ind = [1:LightGraphs.nv(grph)...]
	clusters_found = Array{Int64,1}[]

	while !isempty(current_ind)
		@show "prima $(length(current_ind))"
		e1 = rand(current_ind)
		seen[e1] = true
		cluster = [vmap[e1]]
		S = [e1]

		dir_ref = direction(e1)

		while !isempty(S)
			@show S
			e = pop!(S)
			neighbors = LightGraphs.neighborhood(grph,e,1)
			for neighbor in neighbors
				if !seen[neighbor]
					dir = direction(neighbor)
					if Common.angle_between_directions(dir_ref,dir)<=pi/3
						push!(cluster,vmap[neighbor])
						push!(S,neighbor)
						seen[neighbor]=true
					end
				end
			end
			#dir_ref,_ = Common.LinearFit((Common.apply_matrix(plane.matrix,V))[1:2,union(EV[cluster]...)])
		end
		@show cluster
		push!(clusters_found, cluster)
		setdiff!(current_ind,cluster)
		@show "dopo $(length(current_ind))"

	end

	return clusters_found
end
