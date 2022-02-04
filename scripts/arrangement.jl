println("loading packages... ")

# using PyCall
using ArgParse
using Clipping
using AlphaStructures
using Detection
using Features.DataStructures
using Features.Statistics
using FileManager.JSON
using PyCall

println("packages OK")

function save_dxf_vect3D(
    path_points_fitted,
    path_points_unfitted,
    candidate_points,
    triangles,
    regions,
    filename,
)

    function down_sample(PC::Common.PointCloud,s)
        # default: 3cm distance threshold
        py"""
        import open3d as o3d
        import numpy as np

        def down_sample(points, colors, s):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
            return pcd.voxel_down_sample(s)

        """
        array_points = [c[:] for c in eachcol(PC.coordinates)]
        array_colors = [c[:] for c in eachcol(PC.rgbs)]

        pc_sampled = py"down_sample"(array_points,array_colors,s)

        return PointCloud(permutedims(pc_sampled.points),permutedims(pc_sampled.colors))
    end


    ezdxf = pyimport("ezdxf")
    doc = ezdxf.new()
    msp = doc.modelspace()
    fitted = "fitted"
    unfitted = "unfitted"
    model = "model"
    doc.layers.add(name = fitted, color = 3)
    doc.layers.add(name = unfitted, color = 1)

    py"""
    def add_points_fitted(msp, point):
        msp.add_point(
            point,
            dxfattribs={
            'layer': 'fitted',
            },
        )

    def add_points_unfitted(msp, point):
        msp.add_point(
            point,
            dxfattribs={
            'layer': 'unfitted',
            },
        )
    """

    # leggi i vari file che ti servono e converti
    step = 0.05
    try
        PC = FileManager.source2pc(path_points_fitted)
        pc_fitted = down_sample(PC,step)

        points_fitted = pc_fitted.coordinates #decimare TODO

        for i = 1:size(points_fitted, 2)
            point = points_fitted[:, i]
            pp = (point[1], point[2], point[3])
            py"add_points_fitted"(msp, pp)
        end
    catch
        println("No fitted points")
    end

    try
        PC = FileManager.source2pc(path_points_unfitted)
        pc_unfitted = down_sample(PC,step)
        points_unfitted = pc_unfitted.coordinates #decimare TODO

        for i = 1:size(points_unfitted, 2)
            point = points_unfitted[:, i]
            pp = (point[1], point[2], point[3])
            py"add_points_unfitted"(msp, pp)
        end
    catch
        println("No unfitted points")
    end

    for i = 1:length(regions)
        println("Cluster $i of $(length(regions))")
        region = regions[i]
        faces = triangles[region]
        plane = Common.Plane(candidate_points[:, union(faces...)])

        color = 6
        if Common.abs(Common.dot(plane.normal, [0.0, 0.0, 1.0])) > 0.9
            color = 4
        elseif Common.abs(Common.dot(plane.normal, [0.0, 0.0, 1.0])) < 0.1
            color = 5
        end

        for face in faces
            points = candidate_points[:, face]
            points_array = [c[:] for c in eachcol(points)]
            msp.add_3dface(
                points_array,
                dxfattribs = py"{'layer': $model, 'color': $color}"o,
            )
        end

    end

    doc.saveas(filename)

end


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

function quality_faces(potree, model, output_folder; size_extrusion = 0.02)

    points, edges4faces, faces = model
    tmp_folder = FileManager.mkdir_project(output_folder, "tmp")
    folder_faces = FileManager.mkdir_project(tmp_folder, "FACES")
    dict_faces = Dict{Int,Any}()

    println("PointCloud stored in: ")
    trie = Clipping.potree2trie(potree)
    FileManager.cut_trie!(trie, 3)
    threshold = Features.estimate_threshold(FileManager.source2pc(potree,3), 30)

    for i = 1608:length(faces)
        dict_params = Dict{String,Any}()
        dir = FileManager.mkdir_project(folder_faces, "FACE_$i")

        println("")
        println("======= Processing face $i of $(length(faces))")
        face = faces[i]

        V = points[:, face]

        vmap = collect(1:length(face))
        FV = [copy(vmap)]
        edges = edges4faces[i]
        EV = [
            map(x -> vmap[findfirst(y -> y == x, face)[1]], edge)
            for edge in edges
        ]

        model_extruded = extrude(V, EV, FV, size_extrusion)
        ###
        open(joinpath(dir, "model.txt"), "w") do s
            write(s, "V = $(model_extruded[1])\n\n")
            write(s, "EV = $(model_extruded[2])\n\n")
            write(s, "FV = $(model_extruded[3])\n\n")
        end
        ###

        list_points = Vector{Float64}[]
        n_points_in_volume = clip(trie, potree, model_extruded, list_points)
        points_in_model = hcat(list_points...)

        println("Points in volume: $n_points_in_volume")

        if n_points_in_volume > 10
            area = Common.getArea(V)

            ### get only points nearest the plane
            plane = Common.Plane(V[:, 1:3])
            points_transformed =
                Common.apply_matrix(plane.matrix, points_in_model)

            # z_coords = points_transformed[3,:] # points on plnae XY
            #
            # mu = Statistics.mean(z_coords)
            # std = Statistics.std(z_coords)
            #
            # tokeep = Int[]
            # for i in 1:length(z_coords)
            # 	if z_coords[i]>mu-2*std && z_coords[i]<mu+2*std
            # 		push!(tokeep,i)
            # 	end
            # end

            ### Saving
            FileManager.save_points_txt(
                joinpath(dir, "points_in_model.txt"),
                points_in_model,
            )

            ### compute covered area with alpha shapes if it is possible
            try
                points2D = points_transformed[1:2, :]
                filtration = AlphaStructures.alphaFilter(points2D)
                # threshold = Features.estimate_threshold(points2D, 10)
                _, _, FV = AlphaStructures.alphaSimplex(
                    points2D,
                    filtration,
                    threshold,
                )

                covered_area = Common.getAreaFaces(points2D, FV)
                covered_area_percent = (covered_area / area) * 100
                dict_params["covered_area"] = covered_area
                dict_params["covered_area_percent"] = covered_area_percent
            catch
            end


            dict_params["n_points"] = n_points_in_volume
            dict_params["area"] = area
            dict_params["density"] = n_points_in_volume / area
            dict_params["vertices"] = [c[:] for c in eachcol(V)]
            dict_params["extrusion"] = size_extrusion
            dict_params["points_on_face"] = joinpath(dir, "points_in_model.txt")

            println("Parameters computed")

        end

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
    for (k, v) in dict
        if haskey(v, "covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep, k)
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
        edges = [[face[end], face[1]]]
        for i = 1:length(face)-1
            push!(edges, [face[i], face[i+1]])
        end
        push!(candidate_edges, edges)
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

    println("")
    println("=== SAVINGS ===")
    tokeep = get_valid_faces(dict_faces)

    # clustering valid candidate faces
    println("Clustering coplanar faces...")
    faces = candidate_faces[tokeep]
    edges, triangles, regions =
        Detection.clustering_faces(candidate_points, faces)

    # get polygons
    println("Get polygons...")
    polygons_folder = FileManager.mkdir_project(project_folder, "POLYGONS")
    polygons = Detection.get_polygons(candidate_points, triangles, regions)

    #save boundary polygons
    println("$(length(polygons)) polygons found")
    if !isempty(polygons)

        # bordi poligoni
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

    filename = joinpath(project_folder, "result.dxf")

    path_points_fitted =
        joinpath(project_folder, "POINTCLOUDS/fitted_points.las")

    path_points_unfitted =
        joinpath(project_folder, "POINTCLOUDS/unfitted_points.las")

    save_dxf_vect3D(
        path_points_fitted,
        path_points_unfitted,
        candidate_points,
        triangles,
        regions,
        filename,
    )

end

@time main()
