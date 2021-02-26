using Common
using FileManager
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(source)
aabb = cloudmetadata.tightBoundingBox
NAME_PROJ = "MURI.old"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

hyperplanes, OBBs, alpha_shapes, las_full_inliers = FileManager.read_data_vect2D(folder,NAME_PROJ)

OBBs = [Common.ch_oriented_boundingbox(las2pointcloud(file).coordinates) for file in las_full_inliers]

planes = [Plane(hyperplane.direction,hyperplane.centroid) for hyperplane in hyperplanes]
aabbs = fill( aabb,length(planes))

rV,rEV,rFV = Common.DrawPatches(planes, OBBS)
V,FVs = Common.models_intersection(rV,rEV,rFV)
EVs = [Common.get_boundary_edges(V,FVs[i]) for i in 1:length(FVs)]

centroid = Common.centroid(V)
GL.VIEW( GL.GLExplode(Common.apply_matrix(Lar.t(-centroid...),V),FVs,1.5,1.5,1.5,99,1) );
GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(FVs...)) ]);

GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(EVs...)) ]);
