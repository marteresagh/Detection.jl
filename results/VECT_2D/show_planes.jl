using Common
using FileManager
using Visualization
using Detection

"""
"""
function DrawPlanes(planes::Array{Detection.Hyperplane,1}; box_oriented=true)::Common.LAR
	out = Array{Common.Struct,1}()
	for obj in planes
		plane = Common.Plane(obj.direction,obj.centroid)
		if box_oriented
			box = Common.ch_oriented_boundingbox(obj.inliers.coordinates)
			box.scale .+= 0.02

		else
			box = Common.AABB(obj.inliers.coordinates)
			box.x_max += 0.02
			box.y_max += 0.02
			box.z_max += 0.02
			box.x_min -= 0.02
			box.y_min -= 0.02
			box.z_min -= 0.02
		end
		cell = Common.getmodel(plane,box)
		push!(out, Common.Struct([cell]))
	end
	out = Common.Struct( out )
	V, EV, FV = Common.struct2lar(out)
	return V, EV, FV
end

"""
"""
function planes(PLANES::Array{Detection.Hyperplane,1}, box_oriented = true; affine_matrix = Matrix(Common.I,4,4))

	mesh = []
	for plane in PLANES
		pc = plane.inliers
		V,EV,FV = DrawPlanes([plane]; box_oriented=box_oriented)
		col = Visualization.COLORS[rand(1:12)]
		push!(mesh, Visualization.GLGrid(Common.apply_matrix(affine_matrix,V),FV,col,0.5));
		push!(mesh,	Visualization.points(Common.apply_matrix(affine_matrix,pc.coordinates);color = col,alpha=0.8));
	end

	return mesh
end

"""
"""
function load_connected_components(filename::String)::Common.Cells
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

"""
"""
function get_boundary_models(folders)
	n_planes = length(folders)
	boundary = Common.LAR[]
	for i in 1:n_planes
		#println("$i of $n_planes")
		if isfile(joinpath(folders[i],"execution.probe"))
			V = FileManager.load_points(joinpath(folders[i],"boundary_points3D.txt"))
			EV = load_connected_components(joinpath(folders[i],"boundary_edges.txt"))
			if length(EV)==0
				@show i,folders[i]
			else
				model = (V,EV)
				push!(boundary,model)
			end
		end
	end
	return boundary
end

source = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_IDEALE"
folder_proj = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_ideale"
NAME_PROJ = "PLANES";


INPUT_PC = FileManager.source2pc(source,4)
centroid = Common.centroid(INPUT_PC.coordinates)

folders = Detection.get_plane_folders(folder_proj,NAME_PROJ)
hyperplanes, _ = Detection.get_hyperplanes(folders)

V,EV,FV = DrawPlanes(hyperplanes; box_oriented=true)

Visualization.VIEW([
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
	Visualization.GLGrid(Common.apply_matrix(Common.t(-centroid...),V),FV,Visualization.COLORS[1],0.8)
])
