using Common
using Visualization
using PyCall
using FileManager
using Detection



function box_intersect_face(aabb::Common.AABB, verts_face::Common.Points)
    function point_in_poly(delaunay_model, point)

        py"""
        from scipy.spatial import Delaunay
        import numpy as np

        def point_in_poly(delaunay_model,point):
            simplices = delaunay_model.find_simplex(point)
            return simplices
        """
        check = py"point_in_poly"(delaunay_model, point)

        return check[1] >= 0
    end

    py"""
       from scipy.spatial import Delaunay
       import numpy as np

       def get_delaunay(poly):
           poly = np.array(poly)
           return Delaunay(poly)

       """
    points_box, _, _ = Common.getmodel(aabb)
    points_box = Common.approxVal(8).(points_box)
    delaunay_model = py"get_delaunay"([c[:] for c in eachcol(points_box)])

    n_verts_face = size(verts_face, 2)

    point_in_poly =
        [point_in_poly(delaunay_model, verts_face[:, i]) for i = 1:n_verts_face]
    # @show point_in_poly
    n_points_in_poly = sum(point_in_poly)
    if n_points_in_poly == n_verts_face
        return true
    elseif n_points_in_poly > 0
        return true
    else
        plane = Plane(verts_face[:, 1:3])
        V = Common.box_intersects_plane(aabb, plane.normal, plane.centroid)

        if !isnothing(V) && size(V, 2) > 0
            V = Common.approxVal(8).(V)
            p_face = Common.apply_matrix(plane.matrix, verts_face)
            edges = [[i, i + 1] for i = 1:size(verts_face, 2)-1]
            push!(edges, [size(verts_face, 2), 1])
            p_int = Common.apply_matrix(plane.matrix, V)
            for i = 1:size(p_int, 2)
                if Common.pointInPolygonClassification(p_face, edges)(p_int[
                    :,
                    i,
                ]) != "p_out"
                    return true
                end
            end
        end
    end

    return false
end



source = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\BOXES"

candidate_points, candidate_faces =
    Detection.read_OFF(raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BASIC\vect3D\SEGMENTS\candidate_faces.off")
candidate_edges = Vector{Vector{Int}}[]
for face in candidate_faces
    edges = [[face[end], face[1]]]
    for i = 1:length(face)-1
        push!(edges, [face[i], face[i+1]])
    end
    push!(candidate_edges, edges)
end
# devo fare un traversal per questo
# se interseca il padre interseca forse anche i figli altrimenti skippo

#
# @time for k in keys(trie)
#     aabb = FileManager.las2aabb(trie[k])
#     faces = []
#     for i in 1:length(candidate_faces)
#         face = candidate_faces[i]
#         if box_intersect_face(aabb, candidate_points[:,face])
#             push!(faces, i)
#         end
#     end
#     trie_faces[k] = faces
# end

trie = FileManager.potree2trie(source)
trie_faces = FileManager.Trie{Vector{Int64}}()

for k in keys(trie)
    faces = Int[]
    trie_faces[k] = faces
end

function dfs(trie, trie_faces, points_face, int_face)# due callback: 1 con controllo e 1 senza controllo

    file = trie.value # path to node file
    nodebb = FileManager.las2aabb(file) # aabb of current octree
    inter = box_intersect_face(nodebb, points_face)
    if inter
        # intersecato ma non contenuto
        # alcuni punti ricadono nel modello altri no
        push!(trie_faces.value, int_face)
        for key in collect(keys(trie.children)) # for all children
            dfs(
                trie.children[key],
                trie_faces.children[key],
                points_face,
                int_face,
            )
        end
    end

end

for i = 1:length(candidate_faces)
    points = candidate_points[:, candidate_faces[i]]
    dfs(trie, trie_faces, points, i)
end


# scrivi una visualizzazione per prova
# key_node = "04"
# aabb = FileManager.las2aabb(trie[key_node])
# faces_in_node = trie_faces[key_node]
# V, EV, FV = Common.getmodel(aabb)
#
# # inter = box_intersect_face(aabb, candidate_points[:, candidate_faces[i]])
#
# Visualization.VIEW([
#     [Visualization.GLGrid(candidate_points, candidate_edges[i]) for i in faces_in_node]...
#     Visualization.GLGrid(V, EV, Visualization.COLORS[5])
# ])


function clipping(
    trie,
    trie_faces,
    candidate_points,
    candidate_faces,
    candidate_edges,
    size_extrusion,
)

    points_in_models = [[] for i = 1:length(candidate_faces)]
    model_lists = Vector{Any}(undef, length(candidate_faces))
    #
    for i = 1:length(candidate_faces)
        face = candidate_faces[i]
        V = candidate_points[:, face]
        vmap = collect(1:length(face))
        FV = [copy(vmap)]
        edges = candidate_edges[i]
        EV = [
            map(x -> vmap[findfirst(y -> y == x, face)[1]], edge)
            for edge in edges
        ]

        model_lists[i] = (V, EV, FV)
    end

    println("Faces extrution done ")
    for key in keys(trie)
        @show key
        file = trie[key]
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for count = 1:length(laspoints) # read each point
            if count % 10000 == 0
                println("$count points of $(length(laspoints)) processed")
            end
            laspoint = laspoints[count]
            point = FileManager.xyz(laspoint, header)
            for i in trie_faces[key]
                face = candidate_faces[i]
                V = model_lists[i][1]
                plane = Plane(V[:, 1:3])
                if Common.distance_point2plane(plane.centroid, plane.normal)(
                    point,
                ) < size_extrusion
                    EV = model_lists[i][2]
                    FV = model_lists[i][3]

                    point_transformed = Common.apply_matrix(plane.matrix, point)
                    face_transformed = Common.apply_matrix(plane.matrix, V)
                    if Common.pointInPolygonClassification(
                        face_transformed,
                        EV,
                    )(
                        point_transformed,
                    ) != "p_out"
                        push!(points_in_models[i], point)
                        break
                    end

                end
            end
        end
    end
    return points_in_models, model_lists
end


points_in_models, model_lists = clipping(
    trie,
    trie_faces,
    candidate_points,
    candidate_faces,
    candidate_edges,
    0.05,
)
PC = FileManager.source2pc(source, 1)
for i in 20:40
    @show i
    points = hcat(points_in_models[i]...)
    if size(points, 2) > 2
        Visualization.VIEW([
            Visualization.points(
                PC.coordinates;
                color = Visualization.COLORS[2],
                alpha = 0.7,
            )
            Visualization.points(points)
            Visualization.GLGrid(model_lists[i][1], model_lists[i][2])
        ])
    end
end

for key in keys(trie)
    @show key
    aabb = FileManager.las2aabb(trie[key])
    V, EV, FV = Common.getmodel(aabb)
    Visualization.VIEW([
        Visualization.GLGrid(V, EV, Visualization.COLORS[2])
        Visualization.GLGrid(model_lists[13][1], model_lists[13][2])
    ])
end




aabb = FileManager.las2aabb(trie["55"])
i = 13

inter = box_intersect_face(aabb, candidate_points[:, candidate_faces[i]])
