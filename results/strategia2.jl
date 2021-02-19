using Common
using FileManager
using OrthographicProjection
using AlphaStructures
using Visualization
using Detection

NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

function extract_planes(folder,NAME_PROJ)
	volumes = Volume[]
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			folder_plane = joinpath(root,dir)

			inliers = FileManager.load_points(joinpath(folder_plane,"inliers.txt"))[1:3,:]
			vol = Common.ch_oriented_boundingbox(inliers)
			model = getmodel(vol)

			#OrthographicProjection.segment("C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI", joinpath(joinpath(folder,NAME_PROJ),"$dir.las"), model)
		end
	end
	return true
end

# extract_planes(folder,NAME_PROJ)

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

W,EW = alpha_shapes(folder,NAME_PROJ)
centroid = Common.centroid(W)

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),W)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
])
