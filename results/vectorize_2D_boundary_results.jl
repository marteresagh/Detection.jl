using Common
using FileManager
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "MURI_FULL"
folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"

folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)

function get_boundary_models(folders)
	n_planes = length(folders)
	boundary = Lar.LAR[]
	for i in 1:n_planes
	#	println("$i of $n_planes")

		V = FileManager.load_points(joinpath(folders[i],"boundary_points3D.txt"))
		EV = FileManager.load_connected_components(joinpath(folders[i],"boundary_edges.txt"))
		model = (V,EV)
		if !isempty(EV)

			push!(boundary,model)
		else
			@show i
		end
	end
	return boundary
end

boundary_models = get_boundary_models(folders)
# model = boundary_models[2]

# V = FileManager.load_points("boundary_points3D.txt")
# EV = FileManager.load_connected_components("boundary_edges.txt")

GL.VIEW([
	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V))),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(V)...),V),EV,GL.COLORS[1],0.8),
])



GL.VIEW([
#	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates))),
	[GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),model[1]),model[2],GL.COLORS[1],0.8) for model in boundary_models]...,
])
