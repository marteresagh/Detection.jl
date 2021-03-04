function test(models)
	V,EV = models[1]
	plane = nothing
	for i in 1:10
		@show size(V,2)
		plane = Plane(V)
		P = Common.apply_matrix(plane.matrix,V)[1:2,:]
	    graph = Common.model2graph_edge2edge(V,EV)
		all_clus = []
	    conn_comps = LightGraphs.connected_components(graph)
	    for comp in conn_comps[1:end]
	        subgraph = LightGraphs.induced_subgraph(graph, comp)
	        clus = find_clusters(P,EV, deepcopy(subgraph))
			push!(all_clus,clus)
		end

		EV_rimanenti = simplify_model((V,EV),all_clus)
		V,EV = Lar.simplifyCells(V,EV_rimanenti)
		EW = Lar.lar2cop(EV)
		V,EV = merge_vertices!(V,EW,0.05)
	end

	return V,EV
end

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
		e1 = rand(current_ind)
		seen[e1] = true
		cluster = [vmap[e1]]
		visited = [e1]
		S = [e1]

		dir_ref = direction(e1)

		while !isempty(S)

			e = pop!(S)
			neighbors = LightGraphs.neighborhood(grph,e,1)
			for neighbor in neighbors
				if !seen[neighbor]
					dir = direction(neighbor)
					if Common.angle_between_directions(dir_ref,dir)<=pi/6
						push!(cluster,vmap[neighbor])
						push!(S,neighbor)
						seen[neighbor] = true
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


function simplify_model(model,clusters_found)
	function start_end(model,cluster)
		EV_current = EV[cluster]

		M_2 = Common.K(EV_current)

		S1 = sum(M_2',dims=2)

		inner = [k for k=1:length(S1) if S1[k]==2]
		outer = setdiff(union(EV_current...), inner)
		if length(outer)==2
			return  outer[1], outer[2]
		else
			return nothing, nothing
		end
	end

	EV_rimanenti = Array{Int64,1}[]
	V,EV = model
	for comp in clusters_found
		for cluster in comp
			start_p,end_p = start_end(model,cluster)
			if !isnothing(start_p)
				push!(EV_rimanenti,[start_p,end_p])
			end
		end
	end
	return EV_rimanenti

end



using NearestNeighbors
using SparseArrays
function merge_vertices!(P::Lar.Points, EV::Lar.ChainOp, err=1e-4)
	V = convert(Lar.Points,P')
    vertsnum = size(V, 1)
    edgenum = size(EV, 1)
    newverts = zeros(Int, vertsnum)
    # KDTree constructor needs an explicit array of Float64
    V = Array{Float64,2}(V)
    kdtree = KDTree(permutedims(V))

    # merge congruent vertices
    todelete = []
    i = 1
    for vi in 1:vertsnum
        if !(vi in todelete)
            nearvs = Lar.inrange(kdtree, V[vi, :], err)
            newverts[nearvs] .= i
            nearvs = setdiff(nearvs, vi)
            todelete = union(todelete, nearvs)
            i = i + 1
        end
    end
    nV = V[setdiff(collect(1:vertsnum), todelete), :]

    # merge congruent edges
    edges = Array{Tuple{Int, Int}, 1}(undef, edgenum)
    oedges = Array{Tuple{Int, Int}, 1}(undef, edgenum)
    for ei in 1:edgenum
        v1, v2 = EV[ei, :].nzind

        edges[ei] = Tuple{Int, Int}(sort([newverts[v1], newverts[v2]]))
        oedges[ei] = Tuple{Int, Int}(sort([v1, v2]))
    end
    nedges = union(edges)
    nedges = filter(t->t[1]!=t[2], nedges)
    nedgenum = length(nedges)
    nEV = spzeros(Int8, nedgenum, size(nV, 1))
    # maps pairs of vertex indices to edge index
    etuple2idx = Dict{Tuple{Int, Int}, Int}()
    # builds `edge_map`
    for ei in 1:nedgenum
        nEV[ei, collect(nedges[ei])] .= 1
        etuple2idx[nedges[ei]] = ei
    end
    # return new vertices and new edges
    return convert(Lar.Points,nV'), Lar.cop2lar(nEV)
end
