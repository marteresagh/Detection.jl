println("loading packages... ")

using ArgParse
using Detection
using Common
using Clipping
using PyCall
using DataStructures
using JSON

println("packages OK")

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

function countWithControl(
    params::Clipping.ParametersClipping,
    list_points,
)
    function countWithControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Clipping.Point(laspoint, header)

            @show point.position
            if Common.point_in_polyhedron(point.position,params.model[1],params.model[2],params.model[3]) # if point in model
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

    new_points = hcat(Common.add_zeta_coordinates(V2D,-size_extrusion/2),Common.add_zeta_coordinates(V2D,size_extrusion/2))

    if dim == 3
        V_extruded =
            Common.apply_matrix(Common.inv(plane.matrix), new_points)
    else
        V_extruded = copy(new_points)
    end

    EV_extruded = [EV..., [[i,i+n] for i in 1:n]..., [map(x->x+n,edge) for edge in EV]...]

    FV_extruded = [FV..., [[edge[1],edge[2],edge[2]+n,edge[1]+n] for edge in EV]..., [map(x->x+n,face) for face in FV]...]

    return V_extruded, EV_extruded, FV_extruded
end


function quality_faces(
    potree,
    model,
    output_folder;
    size_extrusion = 0.02,
)

    points, edges4faces, faces = model
    tmp_folder = FileManager.mkdir_project(output_folder, "tmp")
    folder_faces = FileManager.mkdir_project(tmp_folder, "FACES")
    dict_faces = Dict{Int,Any}()

    println("PointCloud stored in: ")
    trie = Clipping.potree2trie(potree)

    for i = 1:length(faces)
        dict_params = Dict{String,Any}()
        dir = FileManager.mkdir_project(folder_faces, "FACE_$i")

        println("")
        println("======= Processing face $i of $(length(faces))")
        face = faces[i]

        V = points[:, face]
        plane = Plane(V)

        vmap = sortperm(face)
        FV = [copy(vmap)]
        edges = edges4faces[i]
        EV = [map(x->vmap[findfirst(y->y==x,face)[1]],edge) for edge in edges]
        # volume = area*size_extrusion
        model = extrude(V, EV, FV, size_extrusion)
        @show model[1]
        @show model[2]
        @show model[3]

        ###
        open(joinpath(dir, "model.txt"), "w") do s
            write(s, "V = $(model[1])\n\n")
            write(s, "EV = $(model[2])\n\n")
            write(s, "FV = $(model[3])\n\n")
        end
        # FileManager.save_points_txt(joinpath(dir, "model.txt"), model[1])
        ###

        list_points = Vector{Float64}[]
        n_points_in_face = clip(trie, potree, model, list_points)
        points_in_model = hcat(list_points...)

        println("Points in face: $n_points_in_face")

        if n_points_in_face > 3 && Common.rank(points_in_model) == 3
            ### Saving
            FileManager.save_points_txt(
                joinpath(dir, "points_in_model.txt"),
                points_in_model,
            )

            area = Common.getArea(V)
            area_points_in_model = Common.getArea(points_in_model)

            dict_params["covered_area"] = area_points_in_model
            dict_params["covered_area_percent"] =
                area_points_in_model / area * 100
            dict_params["area"] = area
            dict_params["density"] = n_points_in_face / area
            dict_params["face_vertex"] = [c[:] for c in eachcol(V)]
            dict_params["extrusion"] = size_extrusion
            dict_params["points_in_face"] = joinpath(dir, "points_in_model.txt")

            println("Parameters computed")

        end

        dict_params["number_points"] = n_points_in_face
        dict_faces[i] = dict_params

        open(joinpath(dir, "faces.js"), "w") do f
            JSON.print(f, dict_faces[i], 4)
        end
        println("=======")

    end

    return dict_faces
end

function get_valid_faces(dict)
    tokeep = []
    for (k,v) in dict
        if v["number_points"] > 10
            if haskey(v,"covered_area_percent") && v["covered_area_percent"] > 50
                push!(tokeep,k)
            end
        end
    end
    return tokeep
end

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "--output", "-o"
        help = "Output folder"
        required = true
        "--potree", "-p"
        help = "Potree"
        required = true
        "--extrusion", "-s"
        help = "size of extrusion"
        arg_type = Float64
        default = 0.2
    end

    return parse_args(s)
end


function main()
    args = parse_commandline()

    project_folder = args["output"]
    potree = args["potree"]
    size_extrusion = args["extrusion"]
    println("== Parameters ==")
    println("Output folder  =>  $project_folder")
    println("Potree  =>  $potree")
    println("Size extrusion  =>  $size_extrusion")


    flush(stdout)

    # read output CGAL
    # final_points, final_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
    CGAL_folder = joinpath(project_folder, "SEGMENTS")
    candidate_points, candidate_faces =
        Detection.read_OFF(joinpath(CGAL_folder, "candidate_faces.off"))
    candidate_edges = Vector{Vector{Int}}[]
    for face in candidate_faces
        edges = [[face[1],face[end]]]
        for i in 1:length(face)-1
            push!(edges,[face[i],face[i+1]])
        end
        push!(candidate_edges, edges )
    end

    model = (candidate_points, candidate_edges, candidate_faces)


    println("")
    println("=== PROCESSING ===")
    dict_faces = quality_faces(
        potree,
        model,
        project_folder;
        size_extrusion = size_extrusion,
    )

    tokeep = get_valid_faces(dict_faces)

    # clustering valid candidate faces
    faces = candidate_faces[tokeep]
    edges, triangles, regions =
        Detection.clustering_faces(candidate_points, faces)

    # get polygons
    polygons_folder = joinpath(project_folder,"POLYGONS")
    polygons = Detection.get_polygons(candidate_points, triangles, regions)

    #save boundary polygons
    if !isempty(polygons)
        Detection.save_boundary_polygons(
            polygons_folder,
            candidate_points,
            polygons,
        )
        FileManager.successful(
            true,
            project_folder;
            filename = "polygons_boundary.probe",
        )
    end

end

@time main()
