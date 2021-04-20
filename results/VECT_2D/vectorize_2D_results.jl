using Common
using FileManager
using Visualization
using Detection

function load_connected_components(filename::String)::Lar.Cells
	EV = Array{Int64,1}[]
	io = open(filename, "r")
	string_conn_comps = readlines(io)
	close(io)

	conn_comps = [tryparse.(Float64,split(string_conn_comps[i], " ")) for i in 1:length(string_conn_comps)]
	for comp in conn_comps
		for i in 1:(length(comp)-1)
			push!(EV, [comp[i],comp[i+1]])
		end
		push!(EV,[comp[end],comp[1]])
	end
	return EV
end

function get_boundary_models(folders)
	n_planes = length(folders)
	boundary = Lar.LAR[]
	for i in 1:n_planes
		#	println("$i of $n_planes")
		if isfile(joinpath(folders[i],"execution.probe"))
			V = FileManager.load_points(joinpath(folders[i],"boundary_points3D.txt"))
			EV = load_connected_components(joinpath(folders[i],"boundary_edges.txt"))
			model = (V,EV)
			push!(boundary,model)
		end
	end
	return boundary
end

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/COLONNA"
INPUT_PC = FileManager.source2pc(source,0)

centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "COLONNA_LOD2"
folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"

folders = Detection.get_plane_folders(folder_proj,NAME_PROJ)

# hyperplanes, _ = get_hyperplanes(folders)
# V,EV,FV = Common.DrawPlanes(hyperplanes; box_oriented=false)
#
# GL.VIEW([
# #	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
# ])
#
# GL.VIEW([
# 	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
# ])

boundary_models = get_boundary_models(folders)

GL.VIEW([
	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates))),
	[GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),model[1]),model[2],GL.COLORS[rand(1:1)],0.8) for model in boundary_models]...,
])

#
# s = 0
# for model in boundary_models
# 	global s
# 	s += size(model[1],2)
# end
