function get_seed_cell(visited_triangles)
    s = -1 # se ritorna questo esce dal loop
    for i = 1:length(visited_triangles)
        if visited_triangles[i] == 0 # se non è mai stata visitata
            s = i
        end
    end
    return s
end

function clustering_preprocess(V, FV)
    triangles = Vector{Int64}[]
    edges = Vector{Int}[]
    dict = DataStructures.Dict{Vector{Int64},Vector{Float64}}()
    for face in FV
        points_face = V[:, face]
        plane = Common.Plane(points_face)
        point_z_zero = Common.apply_matrix(plane.matrix, points_face)[1:2, :]
        tris = Common.delaunay_triangulation(point_z_zero)
        map_triangles = map(x -> face[x], tris)
        for triangle in map_triangles
            push!(triangles, triangle)
            dict[triangle] = plane.normal
            push!(edges, [triangle[1], triangle[2]])
            push!(edges, [triangle[1], triangle[3]])
            push!(edges, [triangle[3], triangle[2]])
        end
    end
    ##
    unique!(edges)
    unique!(triangles)
    return triangles, edges, dict
end

function clustering_adjlist_triangles(triangles, edges)
    copFE = Common.u_coboundary_1(triangles, edges)
    list_adj = Vector{Vector{Int}}(undef, copFE.m)
    # quale triangolo sto studiando
    for i = 1:copFE.m
        adj = Int[]
        c_ld = Common.spzeros(Int8, copFE.m)
        c_ld[i] = 1
        #bordo del triangolo
        bordo = copFE' * c_ld
        for tau in bordo.nzind
            #cobordo del primo
            cobordo = copFE'[tau, :]
            union!(adj, setdiff(cobordo.nzind, i))
        end
        list_adj[i] = adj
    end
    return list_adj
end

function clustering_faces(V, FV)
    ## descrizione del modello
    triangles, edges, dict = clustering_preprocess(V, FV)

    ## grafo adiacenze
    list_adj = clustering_adjlist_triangles(triangles, edges)

    regions = Vector{Int}[]
    visited_triangles = zeros(length(triangles))

    ## main loop
    idx_seed = get_seed_cell(visited_triangles)
    while idx_seed != -1
        region = []
        visited_triangles[idx_seed] = 1
        seeds_face = [copy(idx_seed)]
        push!(region, copy(idx_seed))

        while !isempty(seeds_face)
            seed_face = pop!(seeds_face)
            # println("$seed_face in $seeds_face")
            NN = list_adj[seed_face] #neighbour_of(seed_face)
            for vicino in NN
                # println("sto visitando il vicino $vicino ")
                if visited_triangles[vicino] != 1
                    if Common.abs(Common.dot(
                        dict[triangles[vicino]],
                        dict[triangles[seed_face]],
                    )) > 0.9
                        push!(seeds_face, vicino)
                        push!(region, vicino)
                        visited_triangles[vicino] = 1
                        # println("$vicino visitato")
                    end
                end
            end
            # println("chi sono i seeds ora? $seeds_face")
        end
        push!(regions, region)
        idx_seed = get_seed_cell(visited_triangles)
        # println("$(sum(visited_triangles)) visited of $(length(triangles))")
    end

    return edges, triangles, regions
end

function get_polygons(points, triangles, clusters)
    polygons = []
    for cluster in clusters
        polygon = Common.get_boundary_edges(points, triangles[cluster])
        push!(polygons, polygon)
    end
    return polygons
end

function save_boundary_polygons(folder, points, polygons)

    function lar2cop(CV::Common.Cells)::Common.ChainOp
        I = Int64[]
        J = Int64[]
        Value = Int8[]
        for k = 1:size(CV, 1)
            n = length(CV[k])
            append!(I, k * ones(Int64, n))
            append!(J, CV[k])
            append!(Value, ones(Int64, n))
        end
        return Common.sparse(I, J, Value)
    end


    function cop2lar(cop::Common.ChainOp)::Common.Cells
        [Common.findnz(cop[k, :])[1] for k = 1:size(cop, 1)]
    end

    Threads.@threads for i = 1:length(polygons)
        folder_polygon = FileManager.mkdir_project(folder, "polygon$(i)")
        polygon = polygons[i]
        indx_points = union(polygon...)
        V3D = points[:,indx_points]
        plane = Common.Plane(V3D)
        V2D =  Common.apply_matrix(plane.matrix, V3D)[1:2,:]
        COPEV = lar2cop(polygon)

        EV = cop2lar(COPEV[:, indx_points])


        FileManager.save_points_txt(
            joinpath(folder_polygon, "points2D.txt"),
            V2D,
        )

        FileManager.save_points_txt(
            joinpath(folder_polygon, "points3D.txt"),
            V3D,
        )

        Detection.save_cycles(
            joinpath(folder_polygon, "edges.txt"),
            V3D,
            EV,
        )
    end


end
