using Common
using FileManager
using Visualization
using Detection

folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"
NAME_PROJ = "MURI_LOD3"
folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
boundary, models = FileManager.get_boundary(folders)
centroid = [ 291250.5043433152, 4.630341344699344e6, 106.74835850440863]

#
# function linearized_boundary(models)
# 	out = Array{Lar.Struct,1}()
# 	i = 1
# 	cluss = []
# 	for model in models
# 		@show i
# 		V,EV = model
# 		plane = Plane(V)
# 		V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
# 		(W2D,EW), clus = Detection.linearization(V2D,EV)
# 		W = Common.apply_matrix(Lar.inv(plane.matrix),vcat(W2D,zeros(size(W2D,2))'))
# 		push!(out, Lar.Struct([(W,EW)]))
# 		push!(cluss,clus)
# 		i = i+1
# 	end
# 	out = Lar.Struct(out)
# 	V,EV = Lar.struct2lar(out)
# 	return V,EV, cluss
# end
#
# V_boundary,EV_boundary, cluss = linearized_boundary(full_boundary)
# GL.VIEW([
# 	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V_boundary)')),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V_boundary), EV_boundary)
# ]);
#
# function view_clusters(models,clusters, centroid)
# 	mesh = []
# 	l = length(models)
# 	for i in 1:l
# 		P, EP = models[i]
# 		for clus in clusters[i]
# 			for comp in clus
# 				if !isempty(comp)
# 					col=GL.COLORS[rand(1:12)]
# 					push!(mesh,GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),P), EP[comp], col,1.0))
# 				end
# 			end
# 		end
# 	end
# 	return mesh
# end
#
# GL.VIEW(view_clusters(models,cluss, centroid));
#
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

# mi serve un dizionario di cluster ("1" => [spigoli], "2" => [spigoli],... )
function find_clusters(P::Lar.Points, EV::Lar.Cells, subgraph)
	plane = Plane(P)
	V = Common.apply_matrix(plane.matrix,P)[1:2,:]
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

clusters_found = test(models)

function view_clusters(models, clusters_found, centroid)
	mesh = []
	P, EP = models[1]
	plane = Plane(P)
	for cluster in clusters_found
		for clus in cluster
			col=GL.COLORS[rand(1:12)]
			push!(mesh,GL.GLGrid(Common.apply_matrix(plane.matrix,P)[1:2,:], EP[clus], col,1.0))
		end
	end
	return mesh
end

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(models[1][1]).matrix,models[1][1])[1:2,:])')),
	view_clusters(models,clusters_found, centroid)...,
	GL.GLFrame2
]);


##############DEBUG
clus = clusters_found[1]

function start_end(model,clus)
	V,EV = model
	plane = Plane(V)
	P3D = V[:,union(EV[clus]...)]
	P2D = Common.apply_matrix(plane.matrix,P3D)[1:2,:]
	dir, cent = Common.LinearFit(P2D)

	hmin = +Inf
	hmax = -Inf
	end_p = nothing
	start_p = nothing

	for i in 1:size(P2D,2)
		h = Lar.dot(dir,P2D[:,i])
		if h > hmax
			hmax = h
			end_p =  P2D[:,i]
		elseif h < hmin
			hmin = h
			start_p = P2D[:,i]
		end
	end
	s = Common.matchcolumn(start_p, P2D)
	e = Common.matchcolumn(end_p, P2D)
	s = Common.matchcolumn(P3D[:,s], V)
	e = Common.matchcolumn(P3D[:,e], V)
	return s, e
end

for i in 1:length(clus)
	@show i
	s,e = start_end(models[1],clus[i])
end

s,e = start_end(models[1],clus[11])

function view_clusters(models, cluster, centroid)
	mesh = []
	P, EP = models[1]
	plane = Plane(P)
		for clus in cluster
			col=GL.COLORS[rand(1:12)]
			push!(mesh,GL.GLGrid(Common.apply_matrix(plane.matrix,P)[1:2,:], EP[clus], col,1.0))
		end
	return mesh
end

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(models[1][1]).matrix,models[1][1])[1:2,:])')),
	view_clusters(models,clus, centroid)...,
	GL.GLFrame2
]);
