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


io = open(joinpath("C:/Users/marte/Documents/GEOWEB/TEST/MURI/plane_63781566154549","finite_plane.txt"), "r")
lines = readlines(io)
close(io)

b = [tryparse.(Float64,split(lines[i], " ")) for i in 1:length(lines)]
normal = [b[1][1],b[1][2],b[1][3]]
centroid = normal*b[1][4]

hyperplane = Hyperplane(PointCloud(inliers[1:3,:],inliers[4:6,:]), normal, centroid)
OBB = Volume([b[2][1],b[2][2],b[2][3]],[b[3][1],b[3][2],b[3][3]],[b[4][1],b[4][2],b[4][3]])

################# DEBUG
inliers = hyperplanes[1].inliers.coordinates

obb = Common.ch_oriented_boundingbox(inliers)
# obb = Common.ch_oriented_boundingbox(inliers)

V,EV,FV = Common.getmodel(obb)
GL.VIEW( [
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),inliers)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV)
]);
