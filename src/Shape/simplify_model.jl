"""
linearizazzione della patch planare 3D.
"""
function simplify_model(model::Lar.LAR; par = 0.01, angle = pi/8)::Lar.LAR
	# model = V,EV in 3D space
	V,EV = model
	EV = unique(sort.(EV)) # per togliere un problema nel salvataggio delle componenti. Poi da eliminare
	EV = filter(ev -> length(ev) > 1, EV)
	npoints = size(V,2)
	diff_npoints = npoints

	# porto i punti sul piano 2D
	plane = Plane(V)
	P = Common.apply_matrix(plane.matrix,V)[1:2,:]

	while diff_npoints!=0
		all_clusters_in_model = Array{Array{Int64,1},1}[] #tutti i cluster di spigoli nel modello

		# grafo riferito agli spigoli
	    graph = Common.model2graph_edge2edge(P,EV)
	    conn_comps = Common.biconnected_comps(P,EV)

		# per ogni componente connessa
	    for comp in conn_comps
			# calcolo il sottografo indotto dalla componente connessa
	        subgraph = LightGraphs.induced_subgraph(graph, comp)
			# estraggo catene lineari come collezioni di indici
	        cl_edges = get_cluster_edges((P,EV), subgraph; par=par, angle=angle)
			push!(all_clusters_in_model, cl_edges)
		end


		# costruisco i nuovi spigoli eliminando i punti interni della catena
		new_EV = simplify_edges(EV, all_clusters_in_model)

		if !isempty(new_EV) # se non vuoto procedo
			# optimize V
			#optimize!(P, EV, new_EV, all_clusters_in_model)

			P,EV = Lar.simplifyCells(P,new_EV) #semplifico il modello eliminando i punti non usati
			#optimize!(P_original, P, EV; par = par)

			#* unisco i vertici molto vicini
			P,EV = remove_some_edges!(P,EV; par = par, angle = angle)  # nuovo modello da riutilizzare
		else # altrimenti mi fermo
			break
		end

		# per la condizione di uscita dal loop
		diff_npoints = npoints - size(P,2)
		npoints = size(P,2)

	end
	P3D = Common.apply_matrix(Lar.inv(plane.matrix),vcat(P,zeros(size(P,2))'))

	### components closure
	# points = Int64[]
	# graph = Common.model2graph_edge2edge(P3D,EV)
	# comps = LightGraphs.connected_components(graph)
	# for comp in comps
	# 	ext = Common.get_boundary_points(V,EV[comp])
	# 	if length(ext) == 2
	# 		push!(points,ext...)
	# 		# push!(EV,ext)
	# 	end
	# end
	# T = P[:,points]
	# kdtree = Common.KDTree(T)
	# for p in 1:length(points)
	# 	@show p
	# 	idxs,dists = Common.knn(kdtree, T[:,p], 1, true, i -> i == p)
	# 	push!(EV,[points[p],points[idxs[1]]])
	# end
	return P3D, EV
end

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


function optimize!(P_original::Lar.Points, P::Lar.Points, new_EV::Lar.Cells; par = 0.01)
	V =  [c[:] for c in eachcol(P_original)]
	dict = Dict()
	for i in 1:length(new_EV)
		inliers = Array{Float64,1}[]
		segment = P[:,new_EV[i]]
		push!(inliers, [c[:] for c in eachcol(segment)]...)
		for point in V
			if test_point_on_segment(segment,point; par = par)
				push!(inliers,point)
			end
		end
		dict[i] = hcat(inliers...)
	end

	########################## fino a qui tutto OK #############################
	vertici = union(new_EV...)
	cop = Lar.characteristicMatrix(new_EV)
	I,J,VAL = Lar.findnz(cop)
	for vertice in vertici
		dove = I[Lar.nzrange(cop, vertice)]
		intersection_clusters = [dict[i] for i in dove]
		if length(intersection_clusters)==2
			dir1,cen1 = Common.LinearFit(intersection_clusters[1])
			dir2,cen2 = Common.LinearFit(intersection_clusters[2])

			line1 = [cen1,cen1 + dir1]
			line2 = [cen2,cen2 + dir2]

			new_point = Common.lines_intersection(line1,line2)

			if !isnothing(new_point)
				P[:,vertice] = new_point
			end

		end
	end

end


function optimize!(P::Lar.Points, EV::Lar.Cells, new_EV::Lar.Cells, all_clusters_in_model::Array{Array{Array{Int64,1},1},1})
	clusters_edges = union(all_clusters_in_model...)
	clusters_verts = [union(EV[element]...) for element in clusters_edges]
	vertici = union(new_EV...)
	for vertice in vertici
		dove = issubset.([vertice],clusters_verts)
		intersection_clusters = clusters_verts[dove]
		if length(intersection_clusters)==2
			dir1,cen1 = Common.LinearFit(P[:,intersection_clusters[1]])
			dir2,cen2 = Common.LinearFit(P[:,intersection_clusters[2]])

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
function remove_some_edges!(P::Lar.Points, EP::Lar.Cells; par=1e-4, angle = pi/8)
	graph = Common.model2graph_edge2edge(P,EP)

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


	for i in 1:length(EP)
		ep = EP[i]
		N = setdiff(LightGraphs.neighborhood(graph,i,1),i)

		dist1 = 0.
		dist2 = 0.

		if length(N)==2
			n1 = N[1]
			n2 = N[2]
			dist1 = Lar.norm(P[:,EP[n1][1]]-P[:,EP[n1][2]])
			dist2 = Lar.norm(P[:,EP[n2][1]]-P[:,EP[n2][2]])
			dir1=direction(n1)
			dir2=direction(n2)

			if Common.angle_between_directions(dir1,dir2) <= angle
				dir_ref, cent = Common.LinearFit(P[:,union(EP[N]...)])
				dist_ortho1, dist_ortho2 = dist_ortho(i, dir_ref, cent)
				#dist = Lar.norm(P[:,ep[1]]-P[:,ep[2]])

				if dist_ortho1 <= par && dist_ortho2 <= par
					centroid = Common.centroid(P[:,ep])
					P[:,ep[1]] = centroid
					P[:,ep[2]] = centroid
				end
			end
		end
	end

	return  merge_vertices!(P, EP; err=par)
end


#segment=start,end point lar.points format
function test_point_on_segment(segment,point; par = 0.01)
	direction = segment[:,2]-segment[:,1]
	dist_segment = Lar.norm(direction)
	direction/=dist_segment
	centroid = segment[:,1]
	test_dist = Common.Dist_Point2Line(point,Hyperplane(direction,centroid))<=2*par
	t = Lar.dot(direction,point-centroid)/dist_segment
	test_in_segment = t>=0 && t<=1
	return test_dist && test_in_segment
end
