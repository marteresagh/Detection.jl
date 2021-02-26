using Common
using FileManager
using OrthographicProjection
using Visualization
using Detection


function segment_pointcloud(folder, NAME_PROJ, potree)
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			if dir!=="PLANES"
				folder_plane = joinpath(root,dir)
				# inliers = FileManager.load_points(joinpath(folder_plane,"inliers.txt"))[1:3,:]
				# OBB = Common.ch_oriented_boundingbox(inliers)
				io = open(joinpath(folder_plane,"finite_plane.txt"), "r")
				lines = readlines(io)
				close(io)
				b = [tryparse.(Float64,split(lines[i], " ")) for i in 1:length(lines)]
				OBB = Volume([b[2][1],b[2][2],b[2][3]],[b[3][1],b[3][2],b[3][3]],[b[4][1],b[4][2],b[4][3]])

				model = getmodel(OBB)
				OrthographicProjection.segment("C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI", joinpath(joinpath(folder,NAME_PROJ),"$dir.las"), model)
			end
		end
	end
end


NAME_PROJ = "MURI.old"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"
potree = "C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI"

segment_pointcloud(folder, NAME_PROJ, potree)
