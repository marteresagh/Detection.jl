"""
	get_plane_folders(folder::String,NAME_PROJ::String)

Return all plane folders.
"""
function get_plane_folders(folder::String,NAME_PROJ::String)
	folders = String[]
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			folder_plane = joinpath(root,dir)
			push!(folders, folder_plane)
		end
	end
	return folders
end

"""
	get_hyperplanes(folders::Array{String,1})

Return all hyperplanes described in `folders`.
"""
function get_hyperplanes(folders::Array{String,1})
	hyperplanes = Hyperplane[]
	n_planes = length(folders)
	OBBs = Volume[]
	for i in 1:n_planes
		io = open(joinpath(folders[i],"finite_plane.txt"), "r")
		lines = readlines(io)
		close(io)

		b = [tryparse.(Float64,split(lines[i], " ")) for i in 1:length(lines)]
		normal = [b[1][1],b[1][2],b[1][3]]
		centroid = normal*b[1][4]
		inliers = FileManager.load_points(joinpath(folders[i],"inliers.txt"))

		hyperplane = Hyperplane(PointCloud(inliers[1:3,:],inliers[4:6,:]), normal, centroid)
		OBB = Volume([b[2][1],b[2][2],b[2][3]],[b[3][1],b[3][2],b[3][3]],[b[4][1],b[4][2],b[4][3]])
		push!(hyperplanes,hyperplane)
		push!(OBBs,OBB)
	end
	return hyperplanes, OBBs
end


"""
	save_finite_plane(folder::String, hyperplane::Hyperplane)
"""
function save_finite_plane(folder::String, hyperplane::Hyperplane)
	inliers = hyperplane.inliers.coordinates
	plane = Plane(hyperplane.direction, hyperplane.centroid)

	obb = Common.ch_oriented_boundingbox(inliers)

	extent = obb.scale
	center = obb.position
	euler = obb.rotation

	io = open(joinpath(folder,"finite_plane.txt"),"w")

	# plane
	write(io, "$(plane.a) $(plane.b) $(plane.c) $(plane.d)\n")
	# extent
	write(io, "$(extent[1]) $(extent[2]) $(extent[3])\n")
	# position
	write(io, "$(center[1]) $(center[2]) $(center[3])\n")
	# euler angles
	write(io, "$(euler[1]) $(euler[2]) $(euler[3])\n")

	close(io)

	FileManager.save_points_rgbs_txt(joinpath(folder,"inliers.txt"), hyperplane.inliers)

end
