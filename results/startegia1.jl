function planes_intersection_test(planes, aabb)
	function EV_FV_Planes(planes::Array{Plane,1}, AABB::AABB)
		out = Array{Lar.Struct,1}()
		for plane in planes
			direction = [plane.a,plane.b,plane.c]
			centroid = direction*plane.d
			V = Common.intersectAABBplane(AABB,direction,centroid)
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
	V = convert(Lar.Points, rV');
	GL.VIEW( GL.GLExplode(V,FVs,1.5,1.5,1.5,99,1) );
	#
	#
	GL.VIEW( [GL.GLPoints(convert(Lar.Points,V')),GL.GLGrid(V,union(FVs...)) ]);
end
