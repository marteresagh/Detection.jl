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

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/PROFILE"
INPUT_PC,threshold = Detection.source2pc(source,3)
par = 0.07
#threshold = 2*0.03
failed = 10
N = 100
k = 30
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params,debug = true)
# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)
V,FV = DrawPlanes(hyperplanes, nothing, 0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[12]),
			# GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(V,FV,GL.COLORS[1],1.0)
		])


GL.VIEW([#	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(hyperplanes)...])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])
