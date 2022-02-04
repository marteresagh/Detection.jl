using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features
using DataStructures
using Clipping
using PyCall

# FROM python
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

############## count points on faces

function clip(trie, models, list_points)

    py"""
    from scipy.spatial import Delaunay
    import numpy as np

    def get_delaunay(poly):
        poly = np.array(poly)
        return Delaunay(poly)

    """

    delaunay_model_lists = [ py"get_delaunay"([c[:] for c in eachcol(model[1])]) for model in models ]
    # println("FULL model")
    for k in keys(trie)
        @show k
        file = trie[k]
        countWithControl(delaunay_model_lists, list_points)(file)
    end
end


function countWithControl(delaunay_model_lists, list_points)
    function countWithControl0(file::String)
        header, laspoints = FileManager.read_LAS_LAZ(file) # read file
        for laspoint in laspoints # read each point
            #point = FileManager.xyz(laspoint, header)
            point = Clipping.Point(laspoint, header)
            # @show point.position
            for i in 1:length(delaunay_model_lists)
                delaunay_model = delaunay_model_lists[i]
                if point_in_poly(delaunay_model, point.position) # if point in model
                    push!(list_points[i], point.position)
                    break 
                end
            end
        end
    end
    return countWithControl0
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


planes_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\CASALETTO"

planes = Common.Plane[]
for file in readdir(planes_folder)
    if endswith(file, "txt")
        V = FileManager.load_points(joinpath(planes_folder, file))
        plane = Common.Plane(V)
        push!(planes, plane)
    end
end


function DrawPlanes(
    planes::Array{Detection.Plane,1},
    AABB::Common.AABB,
)::Common.LAR
    out = Array{Common.Struct,1}()
    for plane in planes
        cell = Common.getmodel(plane, AABB)
        push!(out, Common.Struct([cell]))
    end
    out = Common.Struct(out)
    V, EV, FV = Common.struct2lar(out)
    return V, EV, FV
end
function DrawPlanes(plane::Detection.Plane, AABB)
    return DrawPlanes([plane], AABB)
end



potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_CASALETTO"
PC = FileManager.source2pc(potree, 2)
aabb = Common.AABB(PC.coordinates)


hyperplanes1 = Detection.Hyperplane[]
trie = Clipping.potree2trie(potree)
FileManager.cut_trie!(trie, 2)

points_lists = [[] for i in 1:length(planes)]
models = []
for i = 1:length(planes)
        @show i
    plane = planes[i]
    V, EV, FV = DrawPlanes(plane, aabb)
    model_extruded = extrude(V, EV, FV, 0.05)
    push!(models,model_extruded)
end

clip(trie, models, points_lists)
points_in_models = [hcat(points_lists[i]...) for i in 1:length(planes)]

for i in 1:length(planes)
    push!(
        hyperplanes1,
        Detection.Hyperplane(
            PointCloud(points_in_models[i], zeros(3, size(points_in_models[i], 2))),
            planes[i].normal,
            planes[i].centroid,
        ),
    )
end


for i = 1:length(planes)
    @show i
    plane = planes[i]
    V, EV, FV = DrawPlanes(plane, aabb)
    model_extruded = extrude(V, EV, FV, 0.05)
    Visualization.VIEW([
        Visualization.points(PC.coordinates, PC.rgbs),
        Visualization.points(hyperplanes[i].inliers.coordinates, hyperplanes[i].inliers.rgbs),
        Visualization.GLGrid(
            model_extruded[1],
            model_extruded[2],
            Visualization.COLORS[2],
        ),
    ])

end
Detection.save_plane_segments_in_ply(
    hyperplanes1,
    raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_casaletto\vect3D\SEGMENTS\segments2.ply",
)
