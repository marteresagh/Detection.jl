using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features

project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BASIC\boxes_translate"
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\BOXES_TRANSLATE"
PC = FileManager.source2pc(potree, 1)
# read output CGAL

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
    tokeep = Int[]
    for (k, v) in dict
        if haskey(v, "covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep, parse(Int,k))
        end
    end
    return tokeep
end

faces_folder = joinpath(project_folder, "tmp/FACES")
dict_faces = nothing

open(joinpath(faces_folder, "faces.json"), "r") do f
    global dict_faces
    dict_faces = JSON.parse(f)  # parse and transform data
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

for i = 1:2
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
