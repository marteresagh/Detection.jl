using Common
using FileManager
using AlphaStructures
using Visualization
using Detection

function linearized_boundary(models)
	out = Array{Lar.Struct,1}()
	i = 1
	cluss = []
	for model in models
		@show i
		V,EV = model
		plane = Plane(V)
		V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
		(W2D,EW), clus = Detection.linearization(V2D,EV)
		W = Common.apply_matrix(Lar.inv(plane.matrix),vcat(W2D,zeros(size(W2D,2))'))
		push!(out, Lar.Struct([(W,EW)]))
		push!(cluss,clus)
		i = i+1
	end
	out = Lar.Struct(out)
	V,EV = Lar.struct2lar(out)
	return V,EV, cluss
end

V_boundary,EV_boundary, cluss = linearized_boundary(full_boundary)
GL.VIEW([
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V_boundary)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V_boundary), EV_boundary)
]);

function view_clusters(models,clusters, centroid)
	mesh = []
	l = length(models)
	for i in 1:l
		P, EP = models[i]
		for clus in clusters[i]
			for comp in clus
				if !isempty(comp)
					col=GL.COLORS[rand(1:12)]
					push!(mesh,GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),P), EP[comp], col,1.0))
				end
			end
		end
	end
	return mesh
end

GL.VIEW(view_clusters(full_boundary,cluss, centroid));

cluster = test()
