using Common
using FileManager
using OrthographicProjection
using Visualization
using Detection

NAME_PROJ = "MURI.old"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

function extract_planes(folder,NAME_PROJ)
	volumes = Volume[]
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			if dir!=="PLANES"
				folder_plane = joinpath(root,dir)

				inliers = FileManager.load_points(joinpath(folder_plane,"inliers.txt"))[1:3,:]
				vol = Common.ch_oriented_boundingbox(inliers)
				@show vol.scale[3]
				model = getmodel(vol)
				#OrthographicProjection.segment("C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI", joinpath(joinpath(folder,NAME_PROJ),"$dir.las"), model)
			end
		end
	end
	return true
end

extract_planes(folder,NAME_PROJ)
