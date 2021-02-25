using Common
using FileManager
using Visualization

NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"


function planes_intersection_test(planes, aabb)
	function EV_FV_Planes(planes::Array{Plane,1}, AABBs::Array{AABB,1}, u = 0.05)
		out = Array{Lar.Struct,1}()
		for i in  1:length(planes)
			plane = planes[i]
			direction = [plane.a,plane.b,plane.c]
			centroid = direction*plane.d
			AABB = AABBs[i]
			AABB.x_min -= u
			AABB.x_max += u
			AABB.y_min -= u
			AABB.y_max += u
			AABB.z_min -= u
			AABB.z_max += u
			V = Common.box_intersects_plane(AABB,direction,centroid)
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


	V,EV,FV = EV_FV_Planes(planes,aabb)

	copEV = Lar.coboundary_0(EV::Lar.Cells);
	copFE = Lar.coboundary_1(V, FV::Lar.Cells, EV::Lar.Cells);
	W = convert(Lar.Points, V');

	rV, rcopEV, rcopFE = Lar.Arrangement.spatial_arrangement_1( W,copEV,copFE,false)
	#
	triangulated_faces = Lar.triangulate(rV, [rcopEV, rcopFE]);

	FVs = convert(Array{Lar.Cells}, triangulated_faces);
	centroid = Common.centroid(Matrix(rV'))
	V = Common.apply_matrix(Lar.t(-centroid...),convert(Lar.Points, rV'));
	GL.VIEW( GL.GLExplode(V,FVs,1.5,1.5,1.5,99,1) );
	#
	#
	GL.VIEW( [
				GL.GLPoints(convert(Lar.Points,V')),
				GL.GLGrid(V,union(FVs...))
			 ]);
	return V,FVs
end


function test(folder,NAME_PROJ)
	files = FileManager.searchfile(joinpath(joinpath(folder,NAME_PROJ),"PLANES"),".las")
	planes = Plane[]
	AABBs = AABB[]
	for file in files
		h,_ = LasIO.FileIO.load(file)
		if h.records_count > 1000
			PC = FileManager.las2pointcloud(file)
			points = PC.coordinates
			aabb = Common.boundingbox(points)
			plane = Plane(points)
			push!(planes,plane)
			push!(AABBs,aabb)
		end
	end
	return planes, AABBs
end

planes,AABBs = test(folder,NAME_PROJ)
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
cloudmetadata = CloudMetadata(source)
aabb = cloudmetadata.tightBoundingBox

# planes=[Plane([0,1,0.],[0,0,0.]),Plane([0,0.,1],[0,0,5.]),Plane([1,0,0.],[5,0,0.])]
# AABBs=[AABB(10,0,1,0,10,0),AABB(6,0,6,0,6,4),AABB(6,4,10,0,10,4)]
V,FVs = planes_intersection_test(planes[1:20], AABBs)
