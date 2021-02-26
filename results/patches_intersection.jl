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

rV,rEV,rFV = Common.DrawPatches(planes[1], OBBs)
V,FVs = Common.models_intersection(rV,rEV,rFV)
EVs = [Common.get_boundary_edges(V,FVs[i]) for i in 1:length(FVs)]

centroid = Common.centroid(V)
GL.VIEW( GL.GLExplode(Common.apply_matrix(Lar.t(-centroid...),V),FVs,1.5,1.5,1.5,99,1) );
GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(FVs...)) ]);

GL.VIEW( [GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),V)')),GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),union(EVs...)) ]);


V,FV = Common.DrawPlanes(hyperplanes, Common.boundingbox(INPUT_PC.coordinates))

GL.VIEW([
	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates[:,outliers])'),GL.COLORS[2]) ,
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
])
