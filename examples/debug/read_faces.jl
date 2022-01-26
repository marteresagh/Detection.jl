using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features
function view_face(dir, number)
	dir_face = joinpath(dir,"FACE_$number")
	file_model = joinpath(dir_face, "model.txt")
	include(file_model) #V,EV,FV
	file_points = joinpath(dir_face, "points_in_model.txt")
	if isfile(file_points)
		points_in_model = FileManager.load_points(file_points)
		Visualization.VIEW([
		Visualization.GLGrid(V, EV, Visualization.COLORS[12]),
		Visualization.points(points_in_model; color = Visualization.COLORS[2]),
		Visualization.points(PC.coordinates;color = Visualization.COLORS[1], alpha = 0.5),
		])
	end
end
project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\stanza_handmade"
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_IDEALE"
PC = FileManager.source2pc(potree,-1)
# read output CGAL
# final_points, final_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
CGAL_folder = joinpath(project_folder, "SEGMENTS")
candidate_points, candidate_faces =
    Detection.read_OFF(joinpath(CGAL_folder, "candidate_faces.off"))
candidate_edges = Vector{Vector{Int}}[]
for face in candidate_faces
    edges = [[face[end],face[1]]]
    for i in 1:length(face)-1
        push!(edges,[face[i],face[i+1]])
    end
    push!(candidate_edges, edges )
end

model = (candidate_points, candidate_edges, candidate_faces)

function get_valid_faces(dict)
    tokeep = []
    for (k,v) in dict
        if haskey(v,"covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep,k)
        end
	end
    return tokeep
end

faces_folder = joinpath(project_folder, "tmp/FACES")
dict_faces = Dict{Int,Any}()

for dir in readdir(faces_folder)
    idx = parse(Int,split(dir,"_")[2])
    file = joinpath(faces_folder,dir,"faces.js")
	dict = nothing
	if isfile(file)
	    open(file, "r") do f
		    dict = JSON.parse(f)  # parse and transform data
		end
		dict_faces[idx] = dict
	end

end
function get_faces(dict)
    tokeep = []
    for (k,v) in dict
        if haskey(v,"vertices")
			model = v["vertices"]
			points = hcat(model...)
			plane = Common.Plane(points)
			if Common.abs(Common.dot(plane.normal, [1,0.,0.]))>0.8

            	push!(tokeep,k)
			end
        end
	end
    return tokeep
end
tokeep = get_faces(dict_faces)
tokeep = get_valid_faces(dict_faces)


faces = candidate_faces[tokeep]
edges, triangles, regions =
    Detection.clustering_faces(candidate_points, faces)
mesh = []
for i in 1:length(regions)
	push!(mesh, Visualization.GLGrid(candidate_points,triangles[regions[i]],Visualization.COLORS[rand(1:12)]))
end
Visualization.VIEW(mesh)

dir = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\stanza_handmade\tmp\FACES"
for i = 1:12
	view_face(dir, i)
end


using PyCall

function faces2dxf(candidate_points, triangles, regions, filename)

	ezdxf = pyimport("ezdxf")
	doc = ezdxf.new()
	msp = doc.modelspace()

	for i in 1:length(regions)
		str_layer = "PLANE_$i"
		doc.layers.add(name = "$str_layer", color=rand(1:254))
		region = regions[i]
		faces = triangles[region]
		for face in faces
			points = candidate_points[:,face]
			points_array = [c[:] for c in eachcol(points)]
			msp.add_3dface(points_array, dxfattribs=py"{'layer': $str_layer}"o)
		end
		println("regione $i fatta")
	end

	doc.saveas(filename)

end


filename = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\prova.dxf"
faces2dxf(candidate_points, triangles, regions, filename)
