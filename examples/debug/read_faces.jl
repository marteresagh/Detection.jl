using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features

function extrude(V, EV, FV, size_extrusion)
    dim, n = size(V)
    if dim == 3
        plane = Common.Plane(V)
        V2D = Common.apply_matrix(plane.matrix, V)[1:2, :]
    else
        V2D = copy(V)
    end

    new_points = hcat(
        Common.add_zeta_coordinates(V2D, -size_extrusion / 2),
        Common.add_zeta_coordinates(V2D, size_extrusion / 2),
    )

    if dim == 3
        V_extruded = Common.apply_matrix(Common.inv(plane.matrix), new_points)
    else
        V_extruded = copy(new_points)
    end

    EV_extruded = [
        EV...,
        [[i, i + n] for i = 1:n]...,
        [map(x -> x + n, edge) for edge in EV]...,
    ]

    FV_extruded = [
        FV...,
        [[edge[1], edge[2], edge[2] + n, edge[1] + n] for edge in EV]...,
        [map(x -> x + n, face) for face in FV]...,
    ]

    return V_extruded, EV_extruded, FV_extruded
end

function view_face(dir, number)
    dir_face = joinpath(dir, "FACE_$number")
    file_model = joinpath(dir_face, "model.txt")
    include(file_model) #V,EV,FV
    file_points = joinpath(dir_face, "points_in_model.txt")
    Visualization.VIEW([
        Visualization.GLGrid(V, EV, Visualization.COLORS[12]),
        Visualization.points(
            PC.coordinates;
            color = Visualization.COLORS[1],
            alpha = 0.5,
        ),
    ])

    if isfile(file_points)
        points_in_model = FileManager.load_points(file_points)
        Visualization.VIEW([
            Visualization.GLGrid(V, EV, Visualization.COLORS[12]),
            Visualization.points(
                points_in_model;
                color = Visualization.COLORS[2],
            ),
            Visualization.points(
                PC.coordinates;
                color = Visualization.COLORS[1],
                alpha = 0.5,
            ),
        ])
    end
end
dir = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_casaletto\vect3D\tmp\FACES"
view_face(dir, 115)


project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_casaletto\vect3D"
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_CASALETTO"
PC = FileManager.source2pc(potree, 1)
# read output CGAL
# final_points, final_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
CGAL_folder = joinpath(project_folder, "SEGMENTS")
candidate_points, candidate_faces =
    Detection.read_OFF(joinpath(CGAL_folder, "candidate_faces.off"))
candidate_edges = Vector{Vector{Int}}[]
for face in candidate_faces
    edges = [[face[end], face[1]]]
    for i = 1:length(face)-1
        push!(edges, [face[i], face[i+1]])
    end
    push!(candidate_edges, edges)
end

model = (candidate_points, candidate_edges, candidate_faces)

function get_valid_faces(dict)
    tokeep = []
    for (k, v) in dict
        if haskey(v, "covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep, k)
        end
    end
    return tokeep
end

faces_folder = joinpath(project_folder, "tmp/FACES")
dict_faces = Dict{Int,Any}()

for dir in readdir(faces_folder)
    idx = parse(Int, split(dir, "_")[2])
    file = joinpath(faces_folder, dir, "faces.js")
    dict = nothing
    if isfile(file)
        open(file, "r") do f
            dict = JSON.parse(f)  # parse and transform data
        end
        dict_faces[idx] = dict
    end

end
function get_faces(dict::Dict)
    tokeep = []
    for (k, v) in dict
        if haskey(v, "vertices")
            model = v["vertices"]
            points = hcat(model...)
            plane = Common.Plane(points[:, 1:3])
            if Common.abs(Common.dot(plane.normal, [0, 0.0, 1.0])) > 0.5 &&
               plane.centroid[3] <= 0.5
                push!(tokeep, k)
            end
        end
    end
    return tokeep
end

function get_faces(candidate_points, candidate_faces)
    tokeep = []
    for k = 1:length(candidate_faces)
        face = candidate_faces[k]
        points = candidate_points[:, face]

        plane = Common.Plane(points[:, 1:3])

        if Common.abs(Common.dot(plane.normal, [0, 0.0, 1.0])) > 0.5 &&
           plane.centroid[3] <= 0.5
            push!(tokeep, k)
        end
    end
    return tokeep
end
brutte = get_faces(dict_faces)
tokeep = get_valid_faces(dict_faces)

faces = candidate_faces[tokeep]
edges, triangles, regions = Detection.clustering_faces(candidate_points, faces)
mesh = []

for i = 1:length(regions)
    push!(
        mesh,
        Visualization.GLGrid(
            candidate_points,
            triangles[regions[i]],
            Visualization.COLORS[rand(1:12)],
        ),
    )
end

Visualization.VIEW(mesh)
