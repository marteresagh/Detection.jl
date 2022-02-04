using Common
using Visualization
using Detection
using FileManager

folders = Detection.get_plane_folders(raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_casaletto\stanza_casaletto_pipeline\tmp", "PLANES")
planes = Detection.get_hyperplanes(folders)
hyperplanes = planes[1]
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_CASALETTO"
PC = FileManager.source2pc(potree,2)
aabb = Common.AABB(PC.coordinates)
Detection.refine_planes!(hyperplanes)
function DrawPlanes(planes::Array{Detection.Hyperplane,1}; box_oriented=true)::Common.LAR
	out = Array{Common.Struct,1}()
	for obj in planes
		plane = Common.Plane(obj.direction,obj.centroid)
		if box_oriented
			box = Common.ch_oriented_boundingbox(obj.inliers.coordinates)
		else
			box = Common.AABB(obj.inliers.coordinates)
		end
		cell = Common.getmodel(plane,box)
		push!(out, Common.Struct([cell]))
	end
	out = Common.Struct( out )
	V, EV, FV = Common.struct2lar(out)
	return V, EV, FV
end
function DrawPlanes(plane::Detection.Hyperplane; box_oriented=true)
	return DrawPlanes([plane],box_oriented=box_oriented)
end
V,EV,FV = DrawPlanes(hyperplanes)

Visualization.VIEW([
	Visualization.points(PC.coordinates,PC.rgbs)
	Visualization.GLGrid(V,EV,Visualization.COLORS[2])
])
for i in 1:196
	println("piano $i")
	V,EV,FV = DrawPlanes(hyperplanes[i])

	Visualization.VIEW([
	    Visualization.points(PC.coordinates,PC.rgbs)
	    Visualization.GLGrid(V,EV,Visualization.COLORS[2])
	])


end
