using Common
using FileManager
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(source)
aabb = cloudmetadata.tightBoundingBox
NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

hyperplanes, OBBs, alpha_shapes, las_full_inliers = FileManager.read_data_vect2D(folder,NAME_PROJ)

planes = [Plane(hyperplane.direction,hyperplane.centroid) for hyperplane in hyperplanes]
aabbs = fill( aabb,length(planes))

rV,rEV,rFV = Common.DrawPatches(planes, OBBs)
V,FVs = Common.models_intersection(rV,rEV,rFV)

GL.VIEW( GL.GLExplode(V,FVs,1.5,1.5,1.5,99,1) );
GL.VIEW( [GL.GLPoints(convert(Lar.Points,V')),GL.GLGrid(V,union(FVs...)) ]);
