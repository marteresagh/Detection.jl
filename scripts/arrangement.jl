println("loading packages... ")

using ArgParse
using AlphaStructures
using Detection
using Features.DataStructures
using Features.Statistics
using FileManager.JSON
using PyCall

println("packages OK")

# point in polyhedron
function point_in_poly(delaunay_model, point)

    py"""
    from scipy.spatial import Delaunay

    def py_point_in_poly(delaunay_model,point):
        simplices = delaunay_model.find_simplex(point)
        return simplices
    """
    check = py"py_point_in_poly"(delaunay_model, point)
    return check[1] > 0
end

# save DXF
function save_dxf_vect3D(
    path_points_fitted,
    path_points_unfitted,
    candidate_points,
    triangles,
    regions,
    filename,
    size_voxel
)

    function down_sample(PC::Common.PointCloud, s::Float64)
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

        pc_sampled = py"down_sample"(array_points, array_colors, s)

        return Common.PointCloud(
            permutedims(pc_sampled.points),
            permutedims(pc_sampled.colors),
        )
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

    try
        PC = FileManager.source2pc(path_points_fitted)
        pc_fitted = down_sample(PC, size_voxel)
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
        pc_unfitted = down_sample(PC, size_voxel)
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
    println("Clusters: $(length(regions))")
    doc.saveas(filename)
    println("Save done")

end


# Clipping

function dfs(trie, trie_faces, points_face, int_face)

    file = trie.value # path to node file
    nodebb = FileManager.las2aabb(file) # aabb of current octree
    inter = Common.box_intersect_face(nodebb, points_face)
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

function clipping(
    trie,
    trie_faces,
    candidate_points,
    candidate_faces,
    candidate_edges,
    dist2plane,
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

    n_keys = length(keys(trie))
    current_key = 0
    for key in keys(trie)
        current_key += 1

        if current_key % 10 == 0
            println("   $current_key nodes of $n_keys processed")
            flush(stdout)
        end

        # read file
        file = trie[key]
        header, laspoints = FileManager.read_LAS_LAZ(file)
        for count = 1:length(laspoints) # read each point
            # if count % 10000 == 0
            #     println("$count points of $(length(laspoints)) processed")
            # end
            laspoint = laspoints[count]
            point = FileManager.xyz(laspoint, header)
            for i in trie_faces[key]
                face = candidate_faces[i]
                V = model_lists[i][1]
                plane = Common.Plane(V[:, 1:3])
                if Common.distance_point2plane(plane.centroid, plane.normal)(
                    point,
                ) < dist2plane
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


# description of faces
function quality_faces(potree, model, output_folder; dist2plane = 0.02)

    candidate_points, candidate_edges, candidate_faces = model
    tmp_folder = FileManager.mkdir_project(output_folder, "tmp")
    folder_faces = FileManager.mkdir_project(tmp_folder, "FACES")
    n_faces = length(candidate_faces)

    # dict_faces = Dict{Int,Any}() # data structures

    # pointcloud
    println("PointCloud stored in: ")
    trie = FileManager.potree2trie(potree)
    trie_faces = FileManager.Trie{Vector{Int64}}()

    println("")
    println("Create dual trie of intersection face")
    for k in keys(trie)
        faces = Int[]
        trie_faces[k] = faces
    end

    for i = 1:n_faces
        points = candidate_points[:, candidate_faces[i]]
        dfs(trie, trie_faces, points, i)
    end

    println("")
    println("Find points on each face")
    points_in_models, model_lists = clipping(
        trie,
        trie_faces,
        candidate_points,
        candidate_faces,
        candidate_edges,
        dist2plane,
    )

    # FileManager.cut_trie!(trie, 3)
    threshold =
        Features.estimate_threshold(FileManager.source2pc(potree, 3), 30);

    #open file json
    # f = open(joinpath(folder_faces, "faces.json"), "w")
    # println(f, "{")
    # println("")
    dict_faces = Vector{Union{Nothing,Dict}}(nothing,n_faces)
    println("")
    println("Process each face")

    @inbounds for i = 1:n_faces
        dict_params = Dict{String,Any}()

        # if i % 10 == 0
        #     println("   face $i of $(n_faces)")
        # end
        points_in_model = hcat(points_in_models[i]...)
        n_points_in_face = size(points_in_model, 2)

        # println("Points in face: $n_points_in_face")

        V, EV, _ = model_lists[i]
        if n_points_in_face > 10

            area = Common.getArea(V)

            plane = Common.Plane(V[:, 1:3])
            points_transformed =
                Common.apply_matrix(plane.matrix, points_in_model)
            # FileManager.save_points_txt(
            #     joinpath(dir, "points_in_model_$i.txt"),
            #     points_transformed[1:2, :],
            # )
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
                println("No covered area")
            end

            dict_params["n_points"] = n_points_in_face
            dict_params["area"] = area
            dict_params["density"] = n_points_in_face / area
            dict_params["vertices"] = [c[:] for c in eachcol(V)]
            dict_params["dist2plane"] = dist2plane

            # println("Parameters computed")
            dict_faces[i] = dict_params

            # write json
            # print(f, "\"$i\" : ")
            # JSON.print(f, dict_faces[i], 4)
            # i == n_faces || print(f, ",")
            # flush(f)
        end

        # println("=======")

    end
    # print(f, "}")
    # close(f)

    return dict_faces
end

function get_valid_faces(dict)
    tokeep = []
    for i in 1:length(dict)
        v = dict[i]
        if !isnothing(v)
            if haskey(v, "covered_area_percent") && v["covered_area_percent"] > 50
                push!(tokeep, i)
            end
        end
    end
    return tokeep
end

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "--folder", "-o"
            help = "Project folder"
            required = true
        "--potree", "-p"
            help = "Potree"
            required = true
        "--par", "-e"
            help = "Minimum distance of points from face"
            arg_type = Float64
            default = 0.02
        "--voxel", "-v"
            help = "Size voxel for decimation"
            arg_type = Float64
            default = 0.05
    end

    return parse_args(s)
end


function main()
    args = parse_commandline()

    project_folder = args["output"]
    potree = args["potree"]
    dist2plane = args["par"]
    size_voxel = args["voxel"]

    println("== Parameters ==")
    println("Output folder  =>  $project_folder")
    println("Potree  =>  $potree")
    println("Distance to plane, for checking point on face  =>  $dist2plane")
    println("Size voxel for decimation pointcloud in dxf  =>  $size_voxel")
    flush(stdout)


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

    # process faces
    println("")
    println("=== PROCESSING ===")
    flush(stdout)
    dict_faces = quality_faces(
        potree,
        model,
        project_folder;
        dist2plane = dist2plane,
    )

    println("")
    println("=== SAVINGS ===")
    flush(stdout)
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
        size_voxel
    )

end

@time main()
