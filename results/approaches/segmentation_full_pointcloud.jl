using Common
using FileManager
using OrthographicProjection
using Visualization
using Detection


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


NAME_PROJ = "MURI_LOD3"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"
potree = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"

dirs, hyperplanes, OBBs, alpha_shapes, las_full_inliers = read_all_data(folder,NAME_PROJ)
# segment_pointcloud(dirs, hyperplanes, potree, 0.04)
