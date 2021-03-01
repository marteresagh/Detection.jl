using Common
using FileManager
using AlphaStructures
using Visualization
using Detection

function full_boundary(folders)
	n_planes = length(folders)
	for i in 1:n_planes
		println("$i of $n_planes")
		file = joinpath(folders[i],"full_inliers.las")
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

		FileManager.save_points_txt(joinpath(folders[i],"full_boundary_points.txt"), W)
		FileManager.save_connected_components(joinpath(folders[i],"full_boundary_edges.txt"), W, EW)
	end
end

NAME_PROJ = "MURI_LOD3"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

folders, hyperplanes, OBBs, alpha_shapes, las_full_inliers, full_alpha_shapes = read_data_vect2D(folder,NAME_PROJ)

# full_boundary(folders)

function vect2D(full_alpha_shapes, centroid)
	mesh = []
	for shape in full_alpha_shapes
		V,EV = shape
		push!(mesh,GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),EV,GL.COLORS[1],1.0))
	end
	return mesh
end

centroid = Common.centroid(hyperplanes[1].inliers.coordinates)

GL.VIEW(vect2D(full_alpha_shapes, centroid))
