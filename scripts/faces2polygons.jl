println("loading packages... ")

# using PyCall
using ArgParse
using Detection
using Features.DataStructures
using Features.Statistics
using FileManager.JSON

println("packages OK")

function get_valid_faces(dict)
    tokeep = []
    for (k,v) in dict
        if haskey(v,"covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep,k)
        end
    end
    return tokeep
end




faces = candidate_faces[tokeep]
edges, triangles, regions =
    Detection.clustering_faces(candidate_points, faces)
mesh = []
for i in 1:length(regions)
	push!(mesh, Visualization.GLGrid(candidate_points,triangles[regions[i]],Visualization.COLORS[rand(1:12)]))
end
Visualization.VIEW(mesh)
# # get polygons
# polygons_folder = FileManager.mkdir_project(project_folder, "POLYGONS")
# polygons = Detection.get_polygons(candidate_points, triangles, regions)
#
# #save boundary polygons
# if !isempty(polygons)
#     Detection.save_boundary_polygons(
#         polygons_folder,
#         candidate_points,
#         polygons,
#     )
#     FileManager.successful(
#         true,
#         project_folder;
#         filename = "polygons_boundary.probe",
#     )
# end
#
#
#


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

    project_folder = args["output"]
    potree = args["potree"]
    println("== Parameters ==")
    println("Output folder  =>  $project_folder")
    println("Potree  =>  $potree")


    flush(stdout)

    # read output CGAL
    CGAL_folder = joinpath(project_folder, "SEGMENTS")
    candidate_points, candidate_faces =
        Detection.read_OFF(joinpath(CGAL_folder, "candidate_faces.off"))

	candidate_edges = Vector{Vector{Int}}[]
    for face in candidate_faces
        edges = [[face[end],face[1]]]
        for i in 1:length(face)-1
            push!(edges,[face[i],face[i+1]])
        end
        push!(candidate_edges, edges )
    end

    model = (candidate_points, candidate_edges, candidate_faces)


    println("")
    println("=== Read Params ===")
	faces_folder = joinpath(project_folder, "tmp/FACES")
	dict_faces = Dict{Int,Any}()

	for dir in readdir(faces_folder)
	    idx = parse(Int,split(dir,"_")[2])
	    file = joinpath(faces_folder,dir,"faces.js")
		dict = nothing
		if isfile(file)
		    open(file, "r") do f
			    dict = JSON.parse(f)  # parse and transform data
			end
			dict_faces[idx] = dict
		end
	end

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
