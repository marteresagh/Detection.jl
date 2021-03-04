using NearestNeighbors
using SparseArrays

"""
linearizazzione della patch planare 3D.
"""
function symplify_model(model::Lar.LAR; par = 0.01, angle = pi/8)::Lar.LAR
	# model = V,EV in 3D space
	V,EV = model
	npoints = size(V,2)
	diff_npoints = npoints

	while diff_npoints!=0
		all_clusters_in_model = Array{Array{Int64,1},1}[] #tutti i cluster di spigoli nel modello

		# porto i punti sul piano 2D
		plane = Plane(V)
		P = Common.apply_matrix(plane.matrix,V)[1:2,:]

		# grafo riferito agli spigoli
	    graph = Common.model2graph_edge2edge(V,EV)
	    conn_comps = LightGraphs.connected_components(graph)

		# per ogni componente connessa
	    for comp in conn_comps
			# calcolo il sottografo indotto dalla componente connessa
	        subgraph = LightGraphs.induced_subgraph(graph, comp)
			# estraggo catene lineari come collezioni di indici
	        cl_edges = get_cluster_edges((P,EV), subgraph; angle=angle)
			push!(all_clusters_in_model, cl_edges)
		end

		# costruisco i nuovi spigoli eliminando i punti interni della catena
		new_EV = simplify_edges(EV, all_clusters_in_model)
		V,EV = Lar.simplifyCells(V,new_EV) #semplifico il modello eliminando i punti non usati

		# unisco i vertici molto vicini
		V,EV = remove_some_edges!(V,EV; err=par, angle = angle)  # nuovo modello da riutilizzare

		# per la condizione di uscita dal loop
		diff_npoints = npoints - size(V,2)
		npoints = size(V,2)
	end

	return V,EV
end

"""
cerco i clusters nel sottografo corrente (una componente connessa)
"""
function get_cluster_edges(model::Lar.LAR, subgraph; angle = pi/5)::Array{Array{Int64,1},1}
	# model = V,EV in 2D space
	V, EV = model
	grph, vmap = subgraph

	function direction(e)
		inds = EV[vmap[e]]
		dir = V[:, inds[1]] - V[:, inds[2]]
		dir /= Lar.norm(dir)
		return dir
	end

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
					if Common.angle_between_directions(dir_ref,dir)<=angle
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

"""
crea i nuovi spigoli eliminando i vertici interni della catena e creando un unico spigolo che collega i due vertici estremi
"""
function simplify_edges(EV::Lar.Cells, clusters_of_edges::Array{Array{Array{Int64,1},1},1})::Lar.Cells

	function boundary_chain(EV::Lar.Cells)
		M_2 = Common.K(EV)

		S1 = sum(M_2',dims=2)

		inner = [k for k=1:length(S1) if S1[k]==2]
		outer = setdiff(union(EV...), inner)
		return outer
	end

	new_EV = Array{Int64,1}[] # nuovo modello di spigoli

	# per ogni componente  nella componente connessa
	for comp in clusters_of_edges
		# per ogni cluster nella componente
		for cluster in comp
			# catena di spigoli presi in riferimento
			EV_current = EV[cluster]
			# vertici estremi
			verts_extrema = boundary_chain(EV_current)
			if length(verts_extrema)==2
				# nuovo spigolo cotruito
				push!(new_EV,verts_extrema)
			end
		end
	end

	return new_EV
end


"""
unisce i vertici molto vicini
"""
function merge_vertices!(P::Lar.Points, EP::Lar.Cells; err=1e-4)
	EV = Common.K(EP)
	V = convert(Lar.Points,P')
	kdtree = KDTree(P)

    vertsnum = size(V, 1)
    edgenum = size(EV, 1)
    newverts = zeros(Int, vertsnum)

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


# TODO da sistemare
# voglio eliminare gli spigolini inutili
# 1. provare a manterene in un dizionario tutti i punti del cluster o gli spigoli
# 2. aggiornare ad ogni iterazione
# 3. entri nel cluster se la direzione o la distanza dalla linea di entrambi i vertici del segmento è molto piccola
function remove_some_edges!(P::Lar.Points, EP::Lar.Cells; err=1e-4, angle = pi/8)
	graph = Common.model2graph_edge2edge(P,EP)

	function direction(e)
		inds = EP[e]
		dir = P[:, inds[1]] - P[:, inds[2]]
		dir /= Lar.norm(dir)
		return dir
	end

	for i in 1:length(EP)
		ep = EP[i]
		N = setdiff(LightGraphs.neighborhood(graph,i,1),i)

		flag = false
		dist_ref=0.
		if length(N)==2
			n1 = N[1]
			n2 = N[2]
			dist1 = Lar.norm(P[:,EP[n1][1]]-P[:,EP[n1][2]])
			dist2 = Lar.norm(P[:,EP[n2][1]]-P[:,EP[n2][2]])
			dist_ref = min(dist1,dist2)
			dir1=direction(n1)
			dir2=direction(n2)
			if Common.angle_between_directions(dir1,dir2) <= angle
				flag = true
			end
		end

		dist = Lar.norm(P[:,ep[1]]-P[:,ep[2]])
		if flag && dist <= dist_ref/2
			centroid = Common.centroid(P[:,ep])
			P[:,ep[1]] = centroid
			P[:,ep[2]] = centroid
		end

	end

	return merge_vertices!(P, EP; err=err)
end
