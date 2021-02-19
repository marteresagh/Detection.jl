using Common
using FileManager
using Detection
using LightGraphs
using AlphaStructures
using Visualization

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"
INPUT_PC = FileManager.source2pc(source,1)

centroid = Common.centroid(INPUT_PC.coordinates)

NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

function read_data(folder,NAME_PROJ)
	hyperplanes = Hyperplane[]
	out = Array{Lar.Struct,1}()
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			folder_plane = joinpath(root,dir)

			inliers = FileManager.load_points(joinpath(folder_plane,"inliers.txt"))[1:3,:]

			io = open(joinpath(folder_plane,"finite_plane.txt"), "r")
			point = readlines(io)
			close(io)
			b = [tryparse.(Float64,split(point[i], " ")) for i in 1:length(point)]
			plane = b[1]
			normal = [plane[1],plane[2],plane[3]]

			hyperplane = Hyperplane(PointCloud(inliers), normal, plane[4]*normal)
			push!(hyperplanes,hyperplane)

			W = FileManager.load_points(joinpath(folder_plane,"boundary_points.txt"))
			EW = FileManager.load_connected_components(joinpath(folder_plane,"boundary_edges.txt"))
			out = push!(out, Lar.Struct([(W, EW)]))
		end
	end
	out = Lar.Struct(out)
	W,EW = Lar.struct2lar(out)
	return hyperplanes, W, EW
end

hyperplanes, W, EW = read_data(folder,NAME_PROJ)

GL.VIEW([
	Visualization.mesh_planes(hyperplanes,Lar.t(-centroid...))...,
])


V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)

GL.VIEW([
	Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8),
	# GL.GLGrid(W,EW,GL.COLORS[1],1.0),
])

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),W)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
])
