using Common
using FileManager
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(source)
aabb = cloudmetadata.tightBoundingBox
NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

hyperplanes, OBBs, alpha_shapes, las_full_inliers = FileManager.read_data_vect2D(folder,NAME_PROJ)

# OBBs = [Common.ch_oriented_boundingbox(las2pointcloud(file).coordinates) for file in las_full_inliers]

planes = [Plane(hyperplane.direction,hyperplane.centroid) for hyperplane in hyperplanes]
aabbs = fill( aabb,length(planes))

rV,rEV,rFV = Common.DrawPatches(planes, OBBs)
V,FVs = Common.models_intersection(rV,rEV,rFV)
EVs = [Common.get_boundary_edges(V,FVs[i]) for i in 1:length(FVs)]

centroid = Common.centroid(V)
GL.VIEW( GL.GLExplode(Common.apply_matrix(Lar.t(-centroid...),V),FVs,1.5,1.5,1.5,99,1) );
GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(FVs...)) ]);

GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(EVs...)) ]);




################# DEBUG
inliers = hyperplanes[1].inliers.coordinates
centroid = Common.centroid(inliers)

plane = Plane(hyperplanes[1].direction, centroid)
points2D = Common.apply_matrix(plane.matrix,inliers)[1:2,:]
R = Common.basis_minimum_OBB_2D(points2D)
affine_matrix = Lar.approxVal(8).(Common.matrix4(Lar.inv(R))*plane.matrix) # rotation matrix
center_, R = affine_matrix[1:3,4], affine_matrix[1:3,1:3]

V = Common.apply_matrix(Common.matrix4(Lar.inv(R)),Common.apply_matrix(Lar.t(-center_...),inliers))
aabb = Common.boundingbox(V)

center_aabb = [(aabb.x_max+aabb.x_min)/2,(aabb.y_max+aabb.y_min)/2,(aabb.z_max+aabb.z_min)/2]
center = Common.apply_matrix(Common.matrix4(R),center_aabb) + center_
extent = [aabb.x_max - aabb.x_min,aabb.y_max - aabb.y_min, aabb.z_max - aabb.z_min]

obb = Volume(extent,vcat(center...),Common.matrix2euler(R))


# obb = Common.ch_oriented_boundingbox(inliers)

V,EV,FV = Common.getmodel(obb)
GL.VIEW( [
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),inliers)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV)
]);
