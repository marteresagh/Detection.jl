using Common
using FileManager
using Visualization
using Detection

file = "C:/Users/marte/Documents/GEOWEB/TEST/NAVVIS_LOD3/plane_63784339927897/full_inliers.las"
PC = FileManager.las2pointcloud(file)
function outliers(points, par)
	outliers = Int64[]
	tree = Common.KDTree(points)
	idxs = Common.inrange(tree, points, par)
	for i in 1:length(idxs)
		if length(idxs[i])<3
			push!(outliers,i)
		end
	end
	return outliers
end
#######################################
# se troppi punti si possono decimare #
#######################################
if PC.n_points > 3000000
	points = Common.subsample_poisson_disk(PC.coordinates)
	Detection.flushprintln("Decimation: $(size(points,2)) of $(PC.n_points)")
else
	points = PC.coordinates
end
#######################################
#######################################

plane = Plane(points)
T = Common.apply_matrix(plane.matrix,points)[1:2,:]

out = outliers(T, 0.02) #Common.outliers(PC, collect(1:PC.n_points), 30)
V = T[:, setdiff( collect(1:PC.n_points), out)]
