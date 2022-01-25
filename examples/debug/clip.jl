using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features
using DataStructures
using Clipping



project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_ideale\provaprova"
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_IDEALE"



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

k = 422


############## count points on faces

function clip(trie, txtpotreedirs::String, model::Common.LAR, list_points)
    # initialize parameters

    params =
        Clipping.ParametersClipping(txtpotreedirs, "fake.las", model, nothing)

    # proj_folder = splitdir(params.outputfile)[1]

    for potree in params.potreedirs
        params.numFilesProcessed = 0
        traversal_count(potree, trie, params, list_points)
    end

    return params.numPointsProcessed
end

function traversal_count(
    potree::String,
    trie,
    params::Clipping.ParametersClipping,
    list_points,
)
    # println("= ")
    # println("= PROJECT: $potree")
    # println("= ")

    metadata = FileManager.CloudMetadata(potree) # metadata of current potree project
    # trie = Clipping.potree2trie(potree)
    params.numNodes = length(keys(trie))
    # if model contains the whole point cloud ( == 2)
    #	process all files
    # else
    # 	navigate potree

    volume_model = Common.getmodel(Common.AABB(params.model[1]))
    intersection =
        Common.modelsdetection(volume_model, metadata.tightBoundingBox)

    if intersection == 2
        # println("FULL model")
        for k in keys(trie)
            params.numFilesProcessed = params.numFilesProcessed + 1
            # if params.numFilesProcessed % 100 == 0
            #     println(
            #         params.numFilesProcessed,
            #         " files processed of ",
            #         params.numNodes,
            #     )
            # end

            file = trie[k]
            countWithoutControl(params, list_points)(file)

        end
    elseif intersection == 1
        # println("DFS")
        count_dfs(trie, params, list_points)

        # if params.numNodes - params.numFilesProcessed > 0
        #     println("$(params.numNodes-params.numFilesProcessed) file of $(params.numNodes) not processed - out of region of interest")
        # end
    elseif intersection == 0
        # println("OUT OF REGION OF INTEREST")
    end

end


"""
	dfs(trie::DataStructures.Trie{String}, model::Common.LAR)# due callback: 1 con controllo e 1 senza controllo
Depth search first.
"""
function count_dfs(
    trie::DataStructures.Trie{String},
    params::Clipping.ParametersClipping,
    list_points,
)# due callback: 1 con controllo e 1 senza controllo

    file = trie.value # path to node file
    nodebb = FileManager.las2aabb(file) # aabb of current octree
    volume_model = Common.getmodel(Common.AABB(params.model[1]))
    inter = Common.modelsdetection(volume_model, nodebb)

    if inter == 1
        # intersecato ma non contenuto
        # alcuni punti ricadono nel modello altri no
        params.numFilesProcessed = params.numFilesProcessed + 1
        # if params.numFilesProcessed % 100 == 0
        #     println(
        #         params.numFilesProcessed,
        #         " files processed of ",
        #         params.numNodes,
        #     )
        # end

        countWithControl(params, list_points)(file) # update with check
        for key in collect(keys(trie.children)) # for all children
            count_dfs(trie.children[key], params, list_points)
        end
    elseif inter == 2
        # contenuto: tutti i punti del albero sono nel modello
        for k in keys(trie)
            params.numFilesProcessed = params.numFilesProcessed + 1
            # if params.numFilesProcessed % 100 == 0
            #     println(
            #         params.numFilesProcessed,
            #         " files processed of ",
            #         params.numNodes,
            #     )
            # end
            file = trie[k]
            countWithoutControl(params, list_points)(file) # update without check
        end
    end

end

function countWithControl(params::Clipping.ParametersClipping, list_points)
    function countWithControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Clipping.Point(laspoint, header)
            # @show point.position
            if Common.point_in_polyhedron(
                point.position,
                params.model[1],
                params.model[2],
                params.model[3],
            ) # if point in model
                params.numPointsProcessed = params.numPointsProcessed + 1
                push!(list_points, point.position)
            end
        end
    end
    return countWithControl0
end

function countWithoutControl(params::Clipping.ParametersClipping, list_points)
    function countWithoutControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        params.numPointsProcessed =
            params.numPointsProcessed + header.records_count
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Clipping.Point(laspoint, header)
            push!(list_points, point.position)
        end
    end
    return countWithoutControl0
end

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



points, edges4faces, faces = model


println("PointCloud stored in: ")
trie = Clipping.potree2trie(potree)

i = 422


println("")
println("======= Processing face $i of $(length(faces))")
face = faces[i]

V = points[:, face]


vmap = collect(1:length(face))
FV = [copy(vmap)]
edges = edges4faces[i]
EV = [map(x -> vmap[findfirst(y -> y == x, face)[1]], edge) for edge in edges]
# volume = area*size_extrusion
model_extruded = extrude(V, EV, FV, 0.02)
###
# open(joinpath(dir, "model.txt"), "w") do s
#     write(s, "V = $(model_extruded[1])\n\n")
#     write(s, "EV = $(model_extruded[2])\n\n")
#     write(s, "FV = $(model_extruded[3])\n\n")
# end
# FileManager.save_points_txt(joinpath(dir, "model.txt"), model[1])
###

list_points = Vector{Float64}[]
n_points_in_volume = clip(trie, potree, model_extruded, list_points)
points_in_model = hcat(list_points...)
Visualization.VIEW([Visualization.points(points_in_model), Visualization.GLGrid(model_extruded[1],model_extruded[2])])
# println("Points in volume: $n_points_in_volume")
#
if n_points_in_volume > 10 && Common.rank(points_in_model) == 3
    area = Common.getArea(V)

    ### get only points nearest the plane
    plane = Common.Plane(V)
    points_transformed = Common.apply_matrix(plane.matrix, points_in_model)
    Visualization.VIEW([Visualization.GLFrame2,Visualization.points(points_in_model)])
    Visualization.VIEW([Visualization.GLFrame2,Visualization.points(points_transformed)])

    z_coords = points_transformed[3,:] # points on plnae XY

    mu = Statistics.mean(z_coords)
    mystd = Statistics.std(z_coords)

    tokeep = Int[]
    for i in 1:length(z_coords)
        if z_coords[i]>mu-2*mystd && z_coords[i]<mu+2*mystd
            push!(tokeep,i)
        end
    end
    Visualization.VIEW([Visualization.GLFrame2,Visualization.points(points_transformed[:,tokeep])])
    # ### Saving
    # FileManager.save_points_txt(
    #     "points_in_model.txt",
    #     points_in_model[:,tokeep],
    # )
end
