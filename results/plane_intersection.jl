using Common
using Visualization
using LightGraphs

p = Plane([0,0,1.],4*rand(3))
q = Plane([1,0,0.],2*rand(3))
r = Plane([0,1,0.],3*rand(3))
s = Plane(rand(3),-rand(3))

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
		@show cell
		push!(out, Lar.Struct([cell]))
	end
	out = Lar.Struct( out )
	V,EV,FV = Lar.struct2lar(out)
	return V,EV, FV
end

aabb = AABB(10,-10,10,-10,10,-10)
V,EV,FV = EV_FV_Planes([p,q,r,s],aabb)
# V = [1 6  6 1 1 6. 6 4 1; 1 1 6 6 3 3 3 3 3 ;0 0 0 0 -2 -2 6 8 6]
# FV = [[1,3,2,4],[8,7,6,5,9]]
# EV =  [[1,2],[2,3],[3,4],[4,1],[5,6],[7,6],[7,8],[8,9],[5,9]]

# GL.VIEW([
# 	GL.GLGrid(V,FV,GL.COLORS[1],1.0)
# ])

# EV = convert(Array{Array{Int64,1},1}, collect(Set(Common.CAT(map(Common.FV2EV,FV)))))
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
