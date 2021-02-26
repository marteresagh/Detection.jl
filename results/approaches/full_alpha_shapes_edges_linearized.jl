using Common
using FileManager
using AlphaStructures
using Visualization
using Detection

function alpha_shapes(folder,NAME_PROJ)
	files = FileManager.searchfile(joinpath(joinpath(folder,NAME_PROJ),"PLANES"),".las")
	out = Array{Lar.Struct,1}()
	for file in files
		h,_ = LasIO.FileIO.load(file)
		if h.records_count > 1000
			@show "sono qui"
			PC = FileManager.las2pointcloud(file)
			points = PC.coordinates
			plane = Plane(points)
			V = Common.apply_matrix(plane.matrix,points)[1:2,:]

			# 2. applica alpha shape con alpha = threshold
			DT = Common.delaunay_triangulation(V)
			filtration = AlphaStructures.alphaFilter(V,DT);
			threshold = Common.estimate_threshold(V,40)
			_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

			# 3. estrai bordo
			EV_boundary = Common.get_boundary_edges(V,FV)
			w,EW = Lar.simplifyCells(V,EV_boundary)
			#w, EW = Detection.linearization(w,EW)
			W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))

			out = push!(out, Lar.Struct([(W, EW)]))
		end
	end
	out = Lar.Struct(out)
	W,EW = Lar.struct2lar(out)
	return W, EW
end

NAME_PROJ = "MURI.old"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

W,EW = alpha_shapes(folder,NAME_PROJ)
centroid = Common.centroid(W)

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),W)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
])
