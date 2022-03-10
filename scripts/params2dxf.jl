println("loading packages... ")

using ArgParse
using FileManager
using PyCall

println("packages OK")

#
# function vect2dxf(
#     path_points_fitted,
#     path_points_unfitted,
#     candidate_points,
#     triangles,
#     regions,
#     filename,
#     size_voxel,
# )
#
#     function down_sample(PC::Common.PointCloud, s::Float64)
#         py"""
#         import open3d as o3d
#         import numpy as np
#
#         def down_sample(points, colors, s):
#             pcd = o3d.geometry.PointCloud()
#             pcd.points = o3d.utility.Vector3dVector(np.array(points))
#             pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
#             return pcd.voxel_down_sample(s)
#
#         """
#         array_points = [c[:] for c in eachcol(PC.coordinates)]
#         array_colors = [c[:] for c in eachcol(PC.rgbs)]
#
#         pc_sampled = py"down_sample"(array_points, array_colors, s)
#
#         return Common.PointCloud(
#             permutedims(pc_sampled.points),
#             permutedims(pc_sampled.colors),
#         )
#     end
#
#
#     ezdxf = pyimport("ezdxf")
#     doc = ezdxf.new()
#     msp = doc.modelspace()
#     fitted = "fitted"
#     unfitted = "unfitted"
#     model = "model"
#     doc.layers.add(name = fitted, color = 3)
#     doc.layers.add(name = unfitted, color = 1)
#
#     py"""
#     def add_points_fitted(msp, point):
#         msp.add_point(
#             point,
#             dxfattribs={
#             'layer': 'fitted',
#             },
#         )
#
#     def add_points_unfitted(msp, point):
#         msp.add_point(
#             point,
#             dxfattribs={
#             'layer': 'unfitted',
#             },
#         )
#     """
#
#     # leggi i vari file che ti servono e converti
#
#     try
#         PC = FileManager.source2pc(path_points_fitted)
#         pc_fitted = down_sample(PC, size_voxel)
#         points_fitted = pc_fitted.coordinates #decimare TODO
#
#         for i = 1:size(points_fitted, 2)
#             point = points_fitted[:, i]
#             pp = (point[1], point[2], point[3])
#             py"add_points_fitted"(msp, pp)
#         end
#     catch
#         println("No fitted points")
#     end
#
#     try
#         PC = FileManager.source2pc(path_points_unfitted)
#         pc_unfitted = down_sample(PC, size_voxel)
#         points_unfitted = pc_unfitted.coordinates #decimare TODO
#
#         for i = 1:size(points_unfitted, 2)
#             point = points_unfitted[:, i]
#             pp = (point[1], point[2], point[3])
#             py"add_points_unfitted"(msp, pp)
#         end
#     catch
#         println("No unfitted points")
#     end
#
#     for i = 1:length(regions)
#         region = regions[i]
#         faces = triangles[region]
#         plane = Common.Plane(candidate_points[:, union(faces...)])
#
#         color = 6
#         if Common.abs(Common.dot(plane.normal, [0.0, 0.0, 1.0])) > 0.9
#             color = 4
#         elseif Common.abs(Common.dot(plane.normal, [0.0, 0.0, 1.0])) < 0.1
#             color = 5
#         end
#
#         for face in faces
#             points = candidate_points[:, face]
#             points_array = [c[:] for c in eachcol(points)]
#             msp.add_3dface(
#                 points_array,
#                 dxfattribs = py"{'layer': $model, 'color': $color}"o,
#             )
#         end
#
#     end
#     println("Clusters: $(length(regions))")
#     doc.saveas(filename)
#     println("Save done")
#
# end


function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "--output", "-o"
        help = "output filename"
        arg_type = String
        required = true
        "--image", "-i"
        help = "file image"
        arg_type = String
        required = true
        "--segments", "-s"
        help = "file segments"
        arg_type = String
        required = true
        "--pixel"
        help = "pixel image"
        arg_type = String
        required = true
        "--size"
        help = "size image"
        arg_type = String
        required = true
        "--vertex"
        help = "minimum vertex image"
        arg_type = String
        required = true
    end

    return parse_args(s)
end


function main()
    args = parse_commandline()

    output_file = args["output"]
    path_image = args["image"]
    path_segments = args["segments"]
    pixel_ = args["pixel"]
    pixel = tryparse.(Float64, split(pixel_, " "))
    size_ = args["size"]
    size = tryparse.(Float64, split(size_, " "))
    vertex_ = args["vertex"]
    vertex = tryparse.(Float64, split(vertex_, " "))

    println("== Parameters ==")
    println("Output file  =>  $output_file")
    println("Image in  =>  $path_image")
    println("Segments in  =>  $path_segments")
    println("Image params  =>  $pixel, $size, $vertex")
    flush(stdout)


    ezdxf = pyimport("ezdxf")
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    LINES = "lines"
    IMAGE = "image"
    doc.layers.add(name = IMAGE, color = 1)
    doc.layers.add(name = LINES, color = 3)

    my_image_def = doc.add_image_def(
        filename = path_image,
        size_in_pixel = (pixel[1], pixel[2])
    )

    msp.add_image(
        insert = (vertex[1], vertex[2]),
        size_in_units = (size[1], size[2]),
        image_def = my_image_def,
        rotation = 0,
        dxfattribs = py"{'layer': $IMAGE}"o
    )


    segments_verts = readlines(path_segments)
    for verts in segments_verts
        V = tryparse.(Float64, split(verts, " "))
        msp.add_line(
            (V[1], V[2]),
            (V[3], V[4]),
            dxfattribs = py"{'layer': $LINES}"o,
        )
    end


    doc.saveas(output_file)

end

@time main()
