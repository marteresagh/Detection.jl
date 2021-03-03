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

	for i in 1:2
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

clusters_found = test(models)

function view_clusters(models, clusters_found, centroid)
	mesh = []
	P, EP = models[1]
	plane=Plane(P)
	for cluster in clusters_found
		col=GL.COLORS[rand(1:12)]
		push!(mesh,GL.GLGrid(Common.apply_matrix(plane.matrix,P)[1:2,:], EP[cluster], col,1.0))
	end
	return mesh
end

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(models[1][1]).matrix,models[1][1])[1:2,:])')),
	view_clusters(models,clusters_found[1], centroid)...,
	GL.GLFrame2
]);
