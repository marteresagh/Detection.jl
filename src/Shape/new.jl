function test(models)
    V,EV = models[1]
    graph = Common.model2graph_edge2edge(V,EV)
    conn_comps = LightGraphs.connected_components(graph)
    for comp in conn_comps
        search = true
        while search
            subgraph = LightGraphs.induced_subgraph(graph, comp)
            clus = clusters(V,EV, deepcopy(subgraph))

            search = false
        end
    end
	return clus
end

# mi serve un dizionario di cluster ("1" => [spigoli], "2" => [spigoli],... )
function clusters(V::Lar.Points, EV::Lar.Cells, subgraph)
	grph, vmap = subgraph

	e1 = rand(1:LightGraphs.nv(grph))
	seen = zeros(Bool,LightGraphs.nv(grph))
	seen[e1] = true
	cluster = [vmap[e1]]
	S = [e1]
	inds = EV[vmap[e1]]
	dir_ref = V[:, inds[1]]-V[:, inds[2]]
	dir_ref/=Lar.norm(dir_ref)

	while !isempty(S)
		@show S
		e = pop!(S)
		neighbors = LightGraphs.neighborhood(grph,e,1)
		for neighbor in neighbors
			if !seen[neighbor]

				inds = EV[vmap[neighbor]]
				dir = V[:, inds[1]]-V[:, inds[2]]
				dir/=Lar.norm(dir)

				if Common.angle_between_directions(dir_ref,dir)<=pi/3
					push!(cluster,vmap[e])
					push!(S,e)
				end
			end
		end
	end

	return cluster
end
