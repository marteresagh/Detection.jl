println("loading packages... ")

# using PyCall
using ArgParse
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


function get_valid_faces(dict)
    tokeep = []
    for (k,v) in dict
        if haskey(v,"covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep,k)
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
		# DXF
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
