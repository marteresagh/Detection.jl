using Common
using FileManager
using OrthographicProjection
using Visualization
using Detection

################################################################################
# 0.
# Ho identificato i piani con l'algoritmo "plane detection".
# Per ogni piano trovato salvo, in una cartella:
# - "finite_plane.txt" la descrizione del piano (a,b,c,d | ax+by+cz = d) e del suo obb (scala, centro, rotazione),
# - "inliers.txt" tutti i punti che hanno descritto questo piano ad un certo livello di dettaglio dato,
# - "boundary_points.txt","boundary_edges.txt" bordo dell'alpha shapes degli inliers.
################################################################################

folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"
NAME_PROJ = "MURI_LOD3"
potree = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(potree)
tight_bb = cloudmetadata.tightBoundingBox
PC = FileManager.source2pc(potree,0)
centroid = Common.centroid(PC.coordinates)

folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
hyperplanes, OBBs = FileManager.get_hyperplanes(folders)

V_p, EV_p, FV_p = Common.DrawPlanes(hyperplanes; box_oriented=false)
GL.VIEW([
GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V_p)')),
GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V_p), FV_p)
]);

################################################################################
#							PIANI - INTERSEZIONI							   #
################################################################################

planes = [Plane(hyperplane.direction,hyperplane.centroid) for hyperplane in hyperplanes]
aabbs = fill( tight_bb,length(planes))
rV,rEV,rFV = Common.DrawPatches(planes, OBBs)
V,FVs = Common.models_intersection(rV,rEV,rFV)
EVs = [Common.get_boundary_edges(V,FVs[i]) for i in 1:length(FVs)]

GL.VIEW( GL.GLExplode(Common.apply_matrix(Lar.t(-centroid...),V),FVs,1.5,1.5,1.5,99,1) );

GL.VIEW([
GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),
GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(FVs...))
]);

GL.VIEW([
GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),
GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(EVs...))
]);


################################################################################
# 1.
# Data la descrizione del piano e calcolato l'aabb degli inliers
# estraggo tutti i punti quasi appartenenti al piano all'interno dell' aabb
# salvo un file .las "full_inliers.las" nella cartella del piano associato
################################################################################

function segment_pointcloud(folders, hyperplanes, potree, thickness)
	n_planes = length(folders)
	for i in 1:n_planes
		inliers_points = hyperplanes[i].inliers.coordinates
		aabb = Common.boundingbox(inliers_points)
		plane = Plane(hyperplanes[i].direction,hyperplanes[i].centroid)
		model = Common.getmodel(plane, thickness, aabb)
		OrthographicProjection.segment("C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI", joinpath(folders[i],"full_inliers.las"), model)
	end
end

thickness = 0.04
# RIMUOVERE il commento per estrazione degli inliers
# segment_pointcloud(folders, hyperplanes, potree, thickness)

################################################################################
# 2.
# Dati gli inliers totali estratti dall'intera nuvola
# calcolo l'alpha shapes ed estraggo gli spigoli di bordo
# salvo 2 files nella cartella del piano:
# -"full_boundary_points.txt","full_boundary_edges.txt" per la descrizione del bordo.
################################################################################

las_full_inliers = FileManager.get_all_inliers(folders)

function full_boundary(folders::Array{String,1}, las_full_inliers::Array{String,1})
	n_planes = length(folders)
	for i in 1:n_planes
		println("$i of $n_planes")
		file = las_full_inliers[i]
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

# RIMUOVERE il commento per calcolo degli spigoli di bordo
# full_boundary(folders, las_full_inliers)

boundary, full_boundary = FileManager.get_boundary(folders)

function mesh_models(models, centroid)
	# models = [(V,EV),(V,EV),...,(V,EV)]
	mesh = []
	for model in models
		V,EV = model
		col = GL.COLORS[rand(1:12)]
		push!(mesh,GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),EV,col,1.0))
	end
	return mesh
end


GL.VIEW(mesh_models(boundary, centroid))
GL.VIEW(mesh_models(full_boundary, centroid))

################################################################################
# 3.
# Dato il bordo di ogni patch, provare con la linearizzazione
################################################################################
