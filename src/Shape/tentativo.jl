using Visualization
"""
linearizazzione della patch planare 3D.
"""
function simplify_model(model::Lar.LAR; par = 0.01, angle = pi/8)#::Lar.LAR
	# model = V,EV in 3D space
	V,EV = model
	EV = unique(sort.(EV)) # per togliere un problema nel salvataggio delle componenti. Poi da eliminare
	nedges = length(EV)
	diff_nedges = nedges

	dict = Dict()
	# porto i punti sul piano 2D
	plane = Plane(V)
	P = Common.apply_matrix(plane.matrix,V)[1:2,:]

	for i in 1:length(EV)
		dict[EV[i]] = P[:,EV[i]]
	end

	while diff_nedges!=0
		@show "giro"
		@show "1"

		all_clusters_in_model = Array{Array{Int64,1},1}[] #tutti i cluster di spigoli nel modello

		# grafo riferito agli spigoli
	    graph = Common.model2graph_edge2edge(P,EV)
	    conn_comps = LightGraphs.connected_components(graph)

		# per ogni componente connessa
	    for comp in conn_comps
			# calcolo il sottografo indotto dalla componente connessa
	        subgraph = LightGraphs.induced_subgraph(graph, comp)
			# estraggo catene lineari come collezioni di indici
	        cl_edges = get_cluster_edges((P,EV), subgraph; par=par, angle=angle)
			push!(all_clusters_in_model, cl_edges)
		end

		cluss = union(all_clusters_in_model...)
		# dict = update_dict!(P, EV, cluss, dict)
		# costruisco i nuovi spigoli eliminando i punti interni della catena
		EV, dict = simplify_edges(EV, cluss, dict)
		@show "2"
		optimize!(P::Lar.Points, EV::Lar.Cells, dict)
		#* unisco i vertici molto vicini


		#EV,dict = remove_some_edges!(P, EV, dict; par = par, angle = angle)  # nuovo modello da riutilizzare
		@show "3"
		# per la condizione di uscita dal loop
		diff_nedges = nedges - length(EV)
		nedges = length(EV)
	end
#	P,EV = Lar.simplifyCells(P,EV)
	return P, EV, dict #semplifico il modello eliminando i punti non usati
#	return Common.apply_matrix(Lar.inv(plane.matrix),vcat(P,zeros(size(P,2))')), EV ,dict
end

# function update_dict!(P, EV, cluss, dict)
# 	if isempty(dict)
# 		for i in 1:length(cluss)
# 			dict[i] = P[:,union(EV[cluss[i]]...)]
# 		end
# 		return dict
# 	else
# 		dict_tmp = Dict()
# 		for i in 1:length(cluss)
# 			dict_tmp[i] = hcat([dict[j] for j in cluss[i]]...)
# 		end
# 		return dict_tmp
# 	end
# end

"""
cerco i clusters nel sottografo corrente (una componente connessa)
"""
function get_cluster_edges(model::Lar.LAR, subgraph; par = 0.01, angle = pi/8)::Array{Array{Int64,1},1}
	# model = V,EV in 2D space
	V, EV = model
	grph, vmap = subgraph

	function direction(e)
		inds = EV[vmap[e]]
		dir = V[:, inds[1]] - V[:, inds[2]]
		dir /= Lar.norm(dir)
		return dir
	end

	function test_dist(e, direction, centroid)
		inds = EV[vmap[e]]

		v = V[:,inds[1]] - centroid
		p_star = v - Lar.dot(direction,v)*direction
		dist1 = Lar.norm(p_star)

		v = V[:,inds[2]] - centroid
		p_star = v - Lar.dot(direction,v)*direction
		dist2 = Lar.norm(p_star)

		return dist1<par && dist2<par
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
		centroid = V[:,EV[vmap[e1]][1]]

		while !isempty(S)

			e = pop!(S)
			neighbors = LightGraphs.neighborhood(grph,e,1)
			for neighbor in neighbors
				if !seen[neighbor]
					dir = direction(neighbor)
					test = test_dist(neighbor, dir_ref, centroid)
					if Common.angle_between_directions(dir_ref,dir)<=angle || test
						push!(cluster,vmap[neighbor])
						push!(S,neighbor)
						seen[neighbor] = true
						push!(visited,neighbor)
					end
				end
			end
			dir_ref,centroid = Common.LinearFit(V[:,union(EV[cluster]...)])
		end

		push!(clusters_found, cluster)
		setdiff!(current_ind,visited)
	end

	return clusters_found
end


function optimize!(P::Lar.Points, EV::Lar.Cells, dict)
	vertici = union(EV...)
	cop = Lar.characteristicMatrix(EV)
	I,J,VAL = Lar.findnz(cop)
	for vertice in vertici
		dove = I[Lar.nzrange(cop, vertice)]
		@show EV[dove]
		intersection_clusters = [dict[i] for i in dove]
		if length(intersection_clusters)==2
			dir1,cen1 = Common.LinearFit(intersection_clusters[1])
			dir2,cen2 = Common.LinearFit(intersection_clusters[2])

			line1 = [cen1,cen1+dir1]
			line2 = [cen2,cen2+dir2]

			new_point = Common.lines_intersection(line1,line2)
			@assert !isnothing(new_point) "impossible"
			P[:,vertice] = new_point
		end
	end
end

"""
crea i nuovi spigoli eliminando i vertici interni della catena e creando un unico spigolo che collega i due vertici estremi
"""
function simplify_edges(EV::Lar.Cells, cluss::Array{Array{Int64,1},1},dict)
	dict_tmp = Dict()
	function boundary_chain(EV::Lar.Cells)
		M_2 = Common.K(EV)

		S1 = sum(M_2',dims=2)

		inner = [k for k=1:length(S1) if S1[k]==2]
		outer = setdiff(union(EV...), inner)
		return outer
	end

	new_EV = Array{Int64,1}[] # nuovo modello di spigoli

	# per ogni componente  nella componente connessa
	for cluster in cluss
		# catena di spigoli presi in riferimento
		EV_current = EV[cluster]
		# vertici estremi
		verts_extrema = boundary_chain(EV_current)
		if length(verts_extrema)==2
			# nuovo spigolo cotruito
			dict_tmp[verts_extrema] = hcat([dict[ev] for ev in EV_current]...)
			union!(new_EV,[verts_extrema])
		end
	end

	dict = dict_tmp
	return new_EV, dict
end

"""
unisce i vertici molto vicini
"""
function merge_vertices!(P::Lar.Points, EP::Lar.Cells; err=1e-4)
	EV = Common.K(EP)
	V = convert(Lar.Points,P')
	kdtree = Common.KDTree(P)

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
    nEV = Common.spzeros(Int8, nedgenum, size(nV, 1))
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
# 1. provare a manterene in un dizionario tutti i punti del cluster o gli spigoli
# 2. aggiornare ad ogni iterazione
# 3. entri nel cluster se la direzione o la distanza dalla linea di entrambi i vertici del segmento è molto piccola
"""
rimuove spigoli con una certa caratteristica
"""
function remove_some_edges!(P::Lar.Points, EP::Lar.Cells, dict; par=1e-4, angle = pi/8)
	graph = Common.model2graph_edge2edge(P,EP)
	dict_tmp = copy(dict)
	EV = copy(EP)

	function direction(e)
		inds = EP[e]
		dir = P[:, inds[1]] - P[:, inds[2]]
		dir /= Lar.norm(dir)
		return dir
	end

	function dist_ortho(e, direction, centroid)
		inds = EP[e]
		v = P[:,inds[1]] - centroid
		p_star = v - Lar.dot(direction,v)*direction
		dist1 = Lar.norm(p_star)

		v = P[:,inds[2]] - centroid
		p_star = v - Lar.dot(direction,v)*direction
		dist2 = Lar.norm(p_star)

		return dist1, dist2
	end

	todel = Int64[]
	for i in 1:length(EP)
		ep = EP[i] # attenzione che questi cambiano
		N = setdiff(LightGraphs.neighborhood(graph,i,1),i)

		dist1 = 0.
		dist2 = 0.

		if length(N)==2
			n1 = N[1]
			n2 = N[2]
			dist1 = Lar.norm(P[:,EP[n1][1]]-P[:,EP[n1][2]])
			dist2 = Lar.norm(P[:,EP[n2][1]]-P[:,EP[n2][2]])
			dir1 = direction(n1)
			dir2 = direction(n2)

			if Common.angle_between_directions(dir1,dir2) <= angle
				dir_ref, cent = Common.LinearFit(P[:,union(EP[N]...)])
				dist_ortho1, dist_ortho2 = dist_ortho(i, dir_ref, cent)
				#dist = Lar.norm(P[:,ep[1]]-P[:,ep[2]])

				if dist_ortho1 <= par && dist_ortho2 <= par
					push!(todel, i)
					centroid = Common.centroid(P[:,ep])
					P[:,ep[1]] = centroid
					P[:,ep[2]] = centroid
					dict[EP[N[1]]] = hcat(dict[EP[N[1]]],dict[ep])
					dict[EP[N[2]]] = hcat(dict[EP[N[2]]],dict[ep])
				end
			end
		end
	end

	del = []
	graph = Common.model2graph_edge2edge(P,EV)
	for i in todel
		ep = EP[i]
		delete!(dict_tmp,ep)
		N = setdiff(LightGraphs.neighborhood(graph,i,1),i)
		for n in N
			EV[n][EV[n].==ep[1]] .= ep[2]
			push!(del,n)
			if !haskey(dict_tmp,EV[n])
				dict_tmp[EV[n]] = dict[EP[n]]
			end
		end
		graph = Common.model2graph_edge2edge(P,EV)
	end

	EV = EV[ filter(x->!(x in todel), eachindex(EV)) ]
	for i in del
		delete!(dict_tmp,EP[i])
	end

	return EV, dict_tmp
	#return  merge_vertices!(P, EP; err=par)
end
