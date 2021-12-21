using Common
using FileManager
using Visualization
using Detection

function faces2triangles(V, FV)
  FVs = Vector{Vector{Int64}}[]
  for face in FV

    points_face = V[:, face]

    plane = Common.Plane(points_face)

    point_z_zero = Common.apply_matrix(plane.matrix, points_face)[1:2, :]

    triangle = Common.delaunay_triangulation(point_z_zero)
    push!(FVs, map(x -> face[x], triangle))
  end

  return FVs
end


project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BUILDING\FACES"
INPUT_PC = FileManager.source2pc(raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\BUILDING",-1)
V, FV = Detection.read_OFF(joinpath(raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BUILDING", "output_faces.off"))

FVs = faces2triangles(V, FV)

for dir in readdir(project_folder)
	@show dir
	V_model = FileManager.load_points(joinpath(project_folder,dir,"model.txt"))
	del = Common.delaunay_triangulation(V_model)
	# @show V_model
	# @show del
	if isfile(joinpath(project_folder,dir,"points_in_model.txt"))
		println("punti e facce")
		points = FileManager.load_points(joinpath(project_folder,dir,"points_in_model.txt"))
		Visualization.VIEW([
			Visualization.points(INPUT_PC.coordinates,INPUT_PC.rgbs),
			Visualization.points(points;color=Visualization.COLORS[2]),
			Visualization.mesh_color_from_rgb(V_model,del,ones(size(V_model)...);alpha = 0.2),
			Visualization.GLExplode(V,FVs,1.,1.,1.,99,0.2)...
		])
	else
		Visualization.VIEW([
			Visualization.points(INPUT_PC.coordinates,INPUT_PC.rgbs),
			Visualization.mesh_color_from_rgb(V_model,del,ones(size(V_model)...)),
			Visualization.GLExplode(V,FVs,1.,1.,1.,99,0.2)...
		])
	end

end


############# TODO ESTRUSIONE DA RIVEDERE

Visualization.VIEW([
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	Visualization.GLGrid(Common.apply_matrix(Common.t(-centroid...),V),FV,Visualization.COLORS[1],0.8)
])
