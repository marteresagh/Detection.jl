using Common
using Visualization
using Detection
using FileManager
using DataStructures

function read_OFF(filename::String)
	V = nothing
	FV = nothing
	open(filename) do f
		firstline = readline(f)
		if firstline != "OFF"
			throw(ErrorException("Expected \"OFF\" header, got \"$firstline\""))
		end
		n_vertices = nothing
		n_faces = nothing
		n_edges = nothing
		# read header
		while true
			line = readline(f)
			if line == "" && eof(f)
				throw(ErrorException("Unexpected end of file reading off header"))
			elseif startswith(line, "#")

			else
				tokens = split(line)
				n_vertices = parse(Int, tokens[1])
				n_faces = parse(Int, tokens[2])
				n_edges = parse(Int, tokens[3])
				break
			end
		end

		if n_vertices != 0
			V = zeros(3, n_vertices)

			while true
				line = readline(f)
				if line == "" && eof(f)
					throw(ErrorException("Unexpected end of file reading off header"))
				elseif startswith(line, "#")

				elseif line == ""

				else
					points = split(line)
					x = parse(Float64, points[1])
					y = parse(Float64, points[2])
					z = parse(Float64, points[3])
					V[:, 1] = [x, y, z]
					for i = 2:n_vertices
						line = readline(f)
						points = split(line)
						x = parse(Float64, points[1])
						y = parse(Float64, points[2])
						z = parse(Float64, points[3])
						V[:, i] = [x, y, z]
					end
					break
				end
			end

			FV = Vector{Int}[]
			while true
				line = readline(f)
				if line == "" && eof(f)
					throw(ErrorException("Unexpected end of file reading off header"))
				elseif startswith(line, "#")

				elseif line == ""

				else
					face_info = split(line)

					n_verts_face = parse(Int, face_info[1])
					face = Int[]
					for i = 2:n_verts_face+1
						push!(face, parse(Int, face_info[i]) + 1)
					end
					push!(FV, face)

					for i = 2:n_faces
						line = readline(f)
						face_info = split(line)

						n_verts_face = parse(Int, face_info[1])
						face = Int[]
						for i = 2:n_verts_face+1
							push!(face, parse(Int, face_info[i]) + 1)
						end
						push!(FV, face)
					end
					break
				end
			end
		end


	end
	return V, FV
end

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

function get_no_empty_faces(folder)

	dirs = readdir(folder)
	full_dirs = []
	tokeep = []
	number_of_points = zeros(Int,length(dirs))
	for i in 1:length(dirs)
		n_points = 0
		dir = dirs[i]
		i_face = parse(Int,split(dir,"_")[end])
		println("dir: $dir")
		V = FileManager.load_points(joinpath(folder,dir,"model.txt"))
		filename_points = joinpath(folder,dir,"points_in_model.txt")
		if isfile(filename_points)
			points_in_model = FileManager.load_points(filename_points)
			n_points = size(points_in_model,2)
			println("number of points in model: $n_points")


			push!(full_dirs,joinpath(folder,dir))
			push!(tokeep, i_face)

			# tri = Common.delaunay_triangulation(V)
			#
			# Visualization.VIEW([
			#     Visualization.GLGrid(V,tri),
			#     Visualization.points(points_in_model)
			# ])
		end
		number_of_points[i_face] = n_points
	end
	return full_dirs, tokeep, number_of_points
end


function view_faces(dirs)
	out = Array{Common.Struct,1}()
	for i in 1:length(dirs)
		dir = dirs[i]
		println("dir: $dir")
		V = FileManager.load_points(joinpath(dir,"model.txt"))
		filename_points = joinpath(dir,"points_in_model.txt")
		if isfile(filename_points)
			points_in_model = FileManager.load_points(filename_points)
			tri = Common.delaunay_triangulation(V)
			cell = (V,tri)
			push!(out, Common.Struct([cell]))
		end

	end
	out = Common.Struct( out )
	V, tri = Common.struct2lar(out)

	Visualization.VIEW([
	Visualization.GLGrid(V,tri),
	])

end
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\BALL"
PC = FileManager.source2pc(potree,-1)
candidate_faces = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BALL\ball_pipeline\CGAL\candidate_faces.off"

V, FV = read_OFF(candidate_faces)

tokeep = collect(1:length(FV))

folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BALL\ball_pipeline\FACES_old"

dirs_full, tokeep, n_point = get_no_empty_faces(folder)


no_zero_faces = findall(x->x>100,n_point)

# view_faces(dirs_full)


FVs = faces2triangles(V, FV[490:490])

Visualization.VIEW([
	Visualization.points(PC.coordinates,PC.rgbs),
	Visualization.GLExplode(V,FVs,1.,1.,1.,99,1.)...,
	])



dict_faces = Dict{Int,Any}()
dict_params = Dict{String,Any}()
dict_params["number_points"] = 0
dict_params["area"] = 12.4
dict_params["face_vertex"] = [[0.,0.,0.],[1.0,1.,0.],[0.,0.,1]]
dict_params["extrusion"] = 0.2




dict_faces[1] = dict_params
