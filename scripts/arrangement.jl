println("loading packages... ")

using ArgParse
using Detection
using Common
using Clipping
using PyCall
using DataStructures

println("packages OK")

############## count points on faces

function clip(txtpotreedirs::String, model::Common.LAR, list_points)
    # initialize parameters

    params =
        Clipping.ParametersClipping(txtpotreedirs, "fake.las", model, nothing)

    proj_folder = splitdir(params.outputfile)[1]

    for potree in params.potreedirs
        params.numFilesProcessed = 0
        traversal_count(potree, params,list_points)
    end

    println("Processed $(params.numPointsProcessed) points")
    return params.numPointsProcessed
end

function traversal_count(potree::String, params::Clipping.ParametersClipping,list_points)
    println("= ")
    println("= PROJECT: $potree")
    println("= ")

    metadata = FileManager.CloudMetadata(potree) # metadata of current potree project
    trie = Clipping.potree2trie(potree)
    params.numNodes = length(keys(trie))
    # if model contains the whole point cloud ( == 2)
    #	process all files
    # else
    # 	navigate potree

    volume_model = Common.getmodel(Common.AABB(params.model[1]))
    intersection =
        Common.modelsdetection(volume_model, metadata.tightBoundingBox)

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
            countWithoutControl(params, list_points )(file)

        end
    elseif intersection == 1
        println("DFS")
        count_dfs(trie, params, list_points)

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
    params::Clipping.ParametersClipping,
    list_points
)# due callback: 1 con controllo e 1 senza controllo

    py"""
    from scipy.spatial import Delaunay
    import numpy as np

    def get_delaunay(poly):
        poly = np.array(poly)
        return Delaunay(poly)

    """

    delaunay_model = py"get_delaunay"([c[:] for c in eachcol(params.model[1])])
    file = trie.value # path to node file
    nodebb = FileManager.las2aabb(file) # aabb of current octree
    volume_model = Common.getmodel(Common.AABB(params.model[1]))
    inter = Common.modelsdetection(volume_model, nodebb)

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

        countWithControl(delaunay_model, params, list_points)(file) # update with check
        for key in collect(keys(trie.children)) # for all children
            count_dfs(trie.children[key], params, list_points)
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
            countWithoutControl(params, list_points)(file) # update without check
        end
    end

end

function countWithControl(delaunay_model, params::Clipping.ParametersClipping, list_points)
    function countWithControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Clipping.Point(laspoint, header)
            if point_in_poly(delaunay_model, point.position) # if point in model
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


function larModelProduct(modelOne, modelTwo)
    (V, cells1) = modelOne
    (W, cells2) = modelTwo

    vertices = DataStructures.OrderedDict()
    k = 1
    for j = 1:size(V, 2)
        v = V[:, j]
        for i = 1:size(W, 2)
            w = W[:, i]
            id = [v; w]
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
                    push!(cell, vertices[[V[:, vc]; W[:, wc]]])
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
    V2D = Common.apply_matrix(plane.matrix, points)[1:2, :]
    tri = Common.delaunay_triangulation(V2D)
    edges = Common.get_boundary_edges(points, tri)
    modelOne = V2D, edges

    interval = [-size_extrution / 2 size_extrution / 2]
    edge = [[1, 2]]
    modelTwo = interval, edge

    points_extruded, cells = larModelProduct(modelOne, modelTwo)
    points_final =
        Common.apply_matrix(Common.inv(plane.matrix), points_extruded)
    return points_final, cells
end

function point_in_poly(delaunay_model, point)
    py"""
    from scipy.spatial import Delaunay
    import numpy as np

    def point_in_poly(delaunay_model,point):
        simplices = delaunay_model.find_simplex(point)
        return simplices
    """
    check = py"point_in_poly"(delaunay_model, point)
    return check[1] > 0
end

function quality_faces(potree, points, faces, output_folder; size_extrusion = 0.02)
    density_faces = Float64[]
    folder_faces = FileManager.mkdir_project(output_folder,"FACES")
    for i = 1:length(faces)
        dir = FileManager.mkdir_project(folder_faces,"FACE_$i")
        println("Faccia numero $i di $(length(faces))")
        face = faces[i]

        V = points[:, face]
        FV = collect(1:length(face))
        area = Common.getArea(V)
        volume = area*size_extrusion
        model = extrude(V, FV, size_extrusion)
        FileManager.save_points_txt(joinpath(dir,"model.txt"),model[1])

        list_points = Vector{Float64}[]
        n_points_in_face = clip(potree, model, list_points)

        if !isempty(list_points)
            FileManager.save_points_txt(joinpath(dir,"points_in_model.txt"),hcat(list_points...))
        end
        density = n_points_in_face/volume
        push!(density_faces, density)

    end
    return density_faces
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
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	output_folder = args["output"]
    potree = args["potree"]
	println("== Parameters ==")
	println("Output folder  =>  $output_folder")
    println("Potree  =>  $potree")


	flush(stdout)

	# read output CGAL
	points, final_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
	points, candidate_faces = Detection.read_OFF(joinpath(output_folder, "candidate_faces.off"))
	# remove faces
	density_faces = quality_faces(potree, points, final_faces, output_folder; size_extrusion = 0.2)
	density_indices = findall(x-> x > 10000., density_faces)
	faces = candidate_faces[density_indices]
	# clustering candidate faces
	edges, triangles, regions = Detection.clustering_faces(points, faces)
	# get polygons
	polygons = Detection.get_polygons(points, triangles, regions)
	#save boundary polygons
	Detection.save_boundary_polygons(output_folder, points, polygons)

end

@time main()
