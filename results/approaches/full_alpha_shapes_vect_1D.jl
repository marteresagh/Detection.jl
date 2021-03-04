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

V,EV = Detection.test(models)
GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,models[1][1])[1:2,:])')),
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Plane(V).matrix,V)[1:2,:],EV, GL.COLORS[2],1.)
])
