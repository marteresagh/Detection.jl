function clip(txtpotreedirs::String, model::Common.LAR)
    # initialize parameters

    params =
        Clipping.ParametersClipping(txtpotreedirs, "fake.las", model, nothing)

    proj_folder = splitdir(params.outputfile)[1]

    for potree in params.potreedirs
        params.numFilesProcessed = 0
        traversal_count(potree, params)
    end


    println("Processed $(params.numPointsProcessed) points")
    return params.numPointsProcessed
end

function traversal_count(potree::String, params::ParametersClipping)
    println("= ")
    println("= PROJECT: $potree")
    println("= ")

    metadata = CloudMetadata(potree) # metadata of current potree project
    trie = Clipping.potree2trie(potree)
    params.numNodes = length(keys(trie))
    # if model contains the whole point cloud ( == 2)
    #	process all files
    # else
    # 	navigate potree

    intersection =
        Common.modelsdetection(params.model, metadata.tightBoundingBox)

    if intersection == 2
        println("FULL model")
        for k in keys(trie)
            params.numFilesProcessed = params.numFilesProcessed + 1
            if params.numFilesProcessed % 100 == 0
                println(
                    params.numFilesProcessed,
                    " files processed of ",
                    params.numNodes,
                )
            end

            file = trie[k]
            countWithoutControl(params)(file)

        end
    elseif intersection == 1
        println("DFS")
        count_dfs(trie, params)

        if params.numNodes - params.numFilesProcessed > 0
            println("$(params.numNodes-params.numFilesProcessed) file of $(params.numNodes) not processed - out of region of interest")
        end
    elseif intersection == 0
        println("OUT OF REGION OF INTEREST")
    end

end


"""
	dfs(trie::DataStructures.Trie{String}, model::Common.LAR)# due callback: 1 con controllo e 1 senza controllo
Depth search first.
"""
function count_dfs(
    trie::DataStructures.Trie{String},
    params::ParametersClipping,
)# due callback: 1 con controllo e 1 senza controllo

    file = trie.value # path to node file
    nodebb = FileManager.las2aabb(file) # aabb of current octree
    inter = Common.modelsdetection(params.model, nodebb)

    if inter == 1
        # intersecato ma non contenuto
        # alcuni punti ricadono nel modello altri no
        params.numFilesProcessed = params.numFilesProcessed + 1
        if params.numFilesProcessed % 100 == 0
            println(
                params.numFilesProcessed,
                " files processed of ",
                params.numNodes,
            )
        end

        countWithControl(params)(file) # update with check
        for key in collect(keys(trie.children)) # for all children
            count_dfs(trie.children[key], params)
        end
    elseif inter == 2
        # contenuto: tutti i punti del albero sono nel modello
        for k in keys(trie)
            params.numFilesProcessed = params.numFilesProcessed + 1
            if params.numFilesProcessed % 100 == 0
                println(
                    params.numFilesProcessed,
                    " files processed of ",
                    params.numNodes,
                )
            end
            file = trie[k]
            countWithoutControl(params)(file) # update without check
        end
    end

end

function countWithControl(params::ParametersClipping)
    function countWithControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Point(laspoint, header)
            if point_in_poly(params.model[1],point.position) # if point in model
                params.numPointsProcessed = params.numPointsProcessed + 1
            end
        end
    end
    return countWithControl0
end

function countWithoutControl(params::ParametersClipping)
    function countWithoutControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        params.numPointsProcessed =
            params.numPointsProcessed + header.records_count
    end
    return countWithoutControl0
end


function remove_faces!(potree, points, faces)

    remove_faces = Int[]
    for i in 1:length(faces)
        face = faces[i]
        V = points[:,face]
        FV = collect(1:length(face))
        model = get_volume(V,FV)
        n_points_in_face = clip(potree, model)
        if n_points_in_face < 200
            push!(remove_faces, i )
        end
    end

    deleteat!(faces,remove_faces)
end

function larModelProduct( modelOne, modelTwo )
    (V, cells1) = modelOne
    (W, cells2) = modelTwo

    vertices = DataStructures.OrderedDict();
    k = 1
    for j in 1:size(V,2)
       v = V[:,j]
        for i in 1:size(W,2)
          w = W[:,i]
            id = [v;w]
            if haskey(vertices, id) == false
                vertices[id] = k
                k = k + 1
            end
        end
    end

    cells = []
    for c1 in cells1
        for c2 in cells2
            cell = []
            for vc in c1
                for wc in c2
                    push!(cell, vertices[[V[:,vc];W[:,wc]]] )
                end
            end
            push!(cells, cell)
        end
    end

    vertexmodel = []
    for v in keys(vertices)
        push!(vertexmodel, v)
    end
    verts = hcat(vertexmodel...)
    cells = [[v for v in cell] for cell in cells]
    return (verts, cells)
end


function extrude(points, face, size_extrution)
    plane = Common.Plane(points)
    V2D = Common.apply_matrix(plane.matrix,points)[1:2,:]
    modelOne = V2D,face

    interval = [0. size_extrution]
    edge = [[1,2]]
    modelTwo = interval, edge

    points_extruded,cells = larModelProduct( modelOne, modelTwo )
    points_final = Common.apply_matrix(Common.inv(plane.matrix), points_extruded)
    return points_final, cells
end

function point_in_poly(poly,point)
    py"""
    from scipy.spatial import Delaunay
    import numpy as np

    def point_in_poly(poly,point):
        poly = np.array(poly)
        simplices = Delaunay(poly).find_simplex(point)
        print(simplices)
        return simplices
    """

    poly = [c[:] for c in eachcol(poly)]
    check = py"point_in_poly"(poly, point)
    return check[1] > 0
end


point_in_poly(poly,point)
V = [0. 1. 1. 0.;
     0. 0. 1. 1.]
EV = [[1,2,3],[2,3,4]]
modelOne = V,EV

V = [ 0. 1.]
EV = [[1,2]]
modelTwo = V,EV
larModelProduct( modelOne, modelTwo )

V = [0. 1. 1. 0.;
     0. 0. 0. 0.
     0. 0. 1. 1.]
EV = [1,2,3,4]
model = extrude(V, EV, 0.2)

Visualization.VIEW([Visualization.GLGrid(model...)])
