# using Visualization
"""
linearizazzione della patch planare 3D.
"""
function simplify_model(model::Lar.LAR; par = 0.01, angle = pi/8)#::Lar.LAR
	# model = V,EV in 3D space
	V,EV = model
	EV = unique(sort.(EV)) 					# \_ alcuni controlli per avere un modello coerente
	EV = filter(ev -> length(ev) > 1, EV)	# |

	nedges = length(EV)
	diff_nedges = nedges

	dict = Dict()

	# porto i punti sul piano 2D
	plane = Plane(V) # piano identificato dai punti
	P = Common.apply_matrix(plane.matrix,V)[1:2,:]

	# init dict
	for i in 1:length(EV)
		dict[i] = P[:,union(EV[i]...)]
	end

	# println("================START============")
	while diff_nedges != 0
		# println("GIRO------------------")
		all_clusters_in_model = Array{Array{Int64,1},1}[] # tutti i cluster di spigoli nel modello

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

		# costruisco i nuovi spigoli eliminando i punti interni della catena costruita
		EV, dict = simplify_edges(EV, cluss, dict)
		# GL.VIEW([
		# 	GL.GLGrid(P,EV,GL.COLORS[1],0.8),
		# 	[GL.GLPoints(permutedims(dict[k]), GL.COLORS[rand(1:12)]) for k in keys(dict)]...,
		# ])

		# calcolo nuovi punti di intersezione (e modifico P forse non devo modificare P?)
		optimize!(P, EV, dict)

		# rimuovo alcuni spigoli non necessari #TODO da rivedere
	#	EV, dict  = remove_some_edges!(P, EV, dict; par = par, angle = angle)  # nuovo modello da riutilizzare

		# per la condizione di uscita dal loop
		diff_nedges = nedges - length(EV)
		nedges = length(EV)
	end

	#semplifico il modello tenendo solo i punti degli spigoli
	Z,EZ = Lar.simplifyCells(P,EV)
	EZ = filter(ev -> length(ev) > 1, EZ) # controllo

	return Common.apply_matrix(Lar.inv(plane.matrix),vcat(Z,zeros(size(Z,2))')), EZ ,dict
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

"""
calcolo punto di intersezione tra cluster
"""
function optimize!(P::Lar.Points, EV::Lar.Cells, dict)
	vertici = union(EV...)
	cop = Lar.characteristicMatrix(EV)
	I,J,VAL = Lar.findnz(cop)
	for vertice in vertici
		dove = I[Lar.nzrange(cop, vertice)]
		intersection_clusters = [dict[i] for i in dove]
		if length(intersection_clusters)==2


			dir1,cen1 = Common.LinearFit(intersection_clusters[1])
			dir2,cen2 = Common.LinearFit(intersection_clusters[2])

			if Common.angle_between_directions(dir1,dir2) >= pi/8

				line1 = [cen1,cen1+dir1]
				line2 = [cen2,cen2+dir2]

				new_point = Common.lines_intersection(line1,line2)

				@assert !isnothing(new_point) "impossible"
				P[:,vertice] = new_point

			end
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

		outer = [k for k=1:length(S1) if S1[k]==1]
		return outer
	end

	new_EV = Array{Int64,1}[]

	# per ogni componente  nella componente connessa
	i = 0
	for cluster in cluss
		# catena di spigoli presi in riferimento
		EV_current = EV[cluster]
		# vertici estremi
		verts_extrema = boundary_chain(EV_current)
		if length(verts_extrema)==2
			i+=1
			# nuovo spigolo cotruito
			dict_tmp[i] = hcat([dict[ev] for ev in cluster]...)
			push!(new_EV, verts_extrema)
		end
	end
	return new_EV[filter(x-> !isdefined(new_EV,x),1:length(new_EV))],  dict_tmp
end

"""
rimuove spigoli con una certa caratteristica
"""
function remove_some_edges!(P::Lar.Points, EP::Lar.Cells, dict; par=1e-4, angle = pi/8)
	# println("-------------REMOVE SOME EDGES------------------")
	graph = Common.model2graph_edge2edge(P,EP)
	dict_tmp = Dict()
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

	clusters = [[i] for i in 1:length(EP)]
	for i in 1:length(EP)
		ep = EP[i]
		N = setdiff(LightGraphs.neighborhood(graph,i,1),i)

		if length(N)==2
			n1 = N[1]
			n2 = N[2]
			dist1 = Lar.norm(P[:,EP[n1][1]]-P[:,EP[n1][2]])
			dist2 = Lar.norm(P[:,EP[n2][1]]-P[:,EP[n2][2]])
			dir1 = direction(n1)
			dir2 = direction(n2)

			if Common.angle_between_directions(dir1,dir2) <= angle
				cent = Common.centroid(P[:,union(EP[N]...)])
				dir_ref = dir1
				# dir_ref, cent = Common.LinearFit(P[:,union(EP[N]...)])

				dist_ortho1, dist_ortho2 = dist_ortho(i, dir_ref, cent)
				#dist = Lar.norm(P[:,ep[1]]-P[:,ep[2]])

				if dist_ortho1 <= par && dist_ortho2 <= par
					# GL.VIEW([
					# 	GL.GLGrid(P,EV,GL.COLORS[1],0.8),
					# 	GL.GLGrid(P,EV[[i,n1,n2]],GL.COLORS[2],1.0),
					# ])
					#
					# hyperplane = Hyperplane(PointCloud(P[:,union(EP[N]...)]), dir_ref, cent)
					# Z,EZ = Common.DrawLines(hyperplane)
					# GL.VIEW([
					# 	GL.GLGrid(P,EP,GL.COLORS[1],1.0),
					# 	GL.GLPoints(permutedims(P[:,union(EP[N]...)]),GL.RED),
					# 	GL.GLGrid(Z,EZ,GL.COLORS[2],1.0)
					# ])

					push!(clusters, [i,n1,n2])
				end
			end
		end
	end

	for i in 1:length(EP)
		filtro = issubset.([i],clusters)
		daunire = clusters[filtro]
		deleteat!(clusters,filtro)
		push!(clusters,union(daunire...))
	end

	return simplify_edges(EP, clusters, dict)
end
