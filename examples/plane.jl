using Detection
using Visualization
using Common
using FileManager
using Statistics

"""
"""
#TODO da cambiare e farla tipo la line senza triangolazione
function DrawPlane(plane::Hyperplane, AABB::AABB)
	V = intersectAABBplane(AABB,plane.direction,plane.centroid)
	#triangulate vertex projected in plane XY
	FV = delaunay_triangulation(V[1:2,:])
	return V, sort.(FV)
end

"""
"""
function DrawPlanes(planes::Array{Hyperplane,1}, AABB::Union{AABB,Nothing}, u=0.2)
	out = Array{Lar.Struct,1}()
	for obj in planes
		if !isnothing(AABB)
			V = intersectAABBplane(AABB,obj.direction,obj.centroid)
		else
			bb = Lar.boundingbox(obj.inliers.coordinates).+([-u,-u,-u],[u,u,u])
			bb = Common.return_AABB(bb)
			V = Common.intersectAABBplane(bb,obj.direction,obj.centroid)
		end
		#triangulate vertex projected in plane XY
		FV = Common.delaunay_triangulation(V[1:2,:])
		cell = (V,sort.(FV))
		push!(out, Lar.Struct([cell]))
	end
	out = Lar.Struct( out )
	V,FV = Lar.struct2lar(out)
	return V, FV
end

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/CASALETTO"

INPUT_PC,threshold = Detection.source2pc(source,2)
par = 0.07
#threshold = 2*0.03
failed = 100
N = 1000
k = 10
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params,debug = true)
