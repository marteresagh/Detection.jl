using Common
using FileManager
using Visualization

NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"


function planes_intersection_test(planes, aabb)
	function model_planes(planes::Array{Plane,1}, boxes)
		out = Array{Lar.Struct,1}()
		for i in  1:length(planes)
			plane = planes[i]
			direction = plane.normal
			centroid = plane.centroid
			box = boxes[i]
			V = Common.box_intersects_plane(box,direction,centroid)
			#triangulate vertex projected in plane XY
			points2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
			FV = Common.delaunay_triangulation(points2D)
			EV = Common.get_boundary_edges(points2D,FV)
			cell = (V,sort.(EV),[[1:size(V,2)...]])
			push!(out, Lar.Struct([cell]))
		end
		out = Lar.Struct( out )
		V,EV,FV = Lar.struct2lar(out)
		return V,EV, FV
	end

	V,EV,FV = model_planes(planes,aabb)

	copEV = Lar.coboundary_0(EV::Lar.Cells);
	copFE = Lar.coboundary_1(V, FV::Lar.Cells, EV::Lar.Cells);
	W = convert(Lar.Points, V');

	rV, rcopEV, rcopFE = Lar.Arrangement.spatial_arrangement_1( W,copEV,copFE,false)

	return rV, rcopEV, rcopFE
end


function test(folder,NAME_PROJ)
	files = FileManager.searchfile(joinpath(joinpath(folder,NAME_PROJ),"PLANES"),".las")
	planes = Plane[]
	AABBs = AABB[]
	OBBs = Volume[]
	for file in files
		h,_ = LasIO.FileIO.load(file)
		if h.records_count > 1000
			PC = FileManager.las2pointcloud(file)
			points = PC.coordinates
			aabb = Common.boundingbox(points)
			obb = Common.ch_oriented_boundingbox(points)
			plane = Plane(points)
			push!(planes,plane)
			push!(AABBs,aabb)
			push!(OBBs,obb)
		end
	end
	return planes, AABBs, OBBs
end

planes, AABBs, OBBs = test(folder,NAME_PROJ)
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(source)
aabb = cloudmetadata.tightBoundingBox

FULL_Area = fill( aabb,length(planes))
# planes=[Plane([0,1,0.],[0,0,0.]),Plane([0,0.,1],[0,0,5.]),Plane([1,0,0.],[5,0,0.])]
# AABBs=[AABB(10,0,1,0,10,0),AABB(6,0,6,0,6,4),AABB(6,4,10,0,10,4)]
rV, rcopEV, rcopFE = planes_intersection_test(planes, FULL_Area)
triangulated_faces = Lar.triangulate(rV, [rcopEV, rcopFE]);

FVs = convert(Array{Lar.Cells}, triangulated_faces);
indx = findall(x->x==0, length.(FVs))
deleteat!(FVs, indx...)
centroid = Common.centroid(Matrix(rV'))
V = Common.apply_matrix(Lar.t(-centroid...),convert(Lar.Points, rV'));

GL.VIEW( GL.GLExplode(V,FVs,1.5,1.5,1.5,99,1) );
#
#
GL.VIEW( [
			GL.GLPoints(convert(Lar.Points,V')),
			GL.GLGrid(V,union(FVs...))
		 ]);
