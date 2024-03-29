using Common
using FileManager
using Features
using Search
using Detection
using Visualization
using AlphaStructures

function DrawPlanes(
    planes::Array{Detection.Hyperplane,1};
    box_oriented = true,
)::Common.LAR
    out = Array{Common.Struct,1}()
    for obj in planes
        plane = Common.Plane(obj.direction, obj.centroid)
        if box_oriented
            box = Common.ch_oriented_boundingbox(obj.inliers.coordinates)
        else
            box = Common.AABB(obj.inliers.coordinates)
        end
        cell = Common.getmodel(plane, box)
        push!(out, Common.Struct([cell]))
    end
    out = Common.Struct(out)
    V, EV, FV = Common.struct2lar(out)
    return V, EV, FV
end
function DrawPlanes(plane::Detection.Hyperplane; box_oriented = true)
    return DrawPlanes([plane], box_oriented = box_oriented)
end

function DrawPlanes(planes::Array{Detection.Hyperplane,1}, box)::Common.LAR
    out = Array{Common.Struct,1}()
    for obj in planes
        @show "plane"
        plane = Common.Plane(obj.direction, obj.centroid)
        cell = Common.getmodel(plane, box)
        push!(out, Common.Struct([cell]))
    end
    out = Common.Struct(out)
    V, EV, FV = Common.struct2lar(out)
    return V, EV, FV
end
function DrawPlanes(plane::Detection.Hyperplane, box)
    return DrawPlanes([plane], box)
end

# ======================================================================
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/STANZA_IDEALE\\"
source = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BASIC\boxes_translate.las"
INPUT_PC = FileManager.source2pc(source, 2)
aabb = Common.AABB(INPUT_PC.coordinates)
aabb.x_max += 0.1
aabb.y_max += 0.1
aabb.z_max += 0.1
aabb.x_min -= 0.1
aabb.y_min -= 0.1
aabb.z_min -= 0.1
# user parameters
par = 0.02
failed = 100
N = 40
k = 30

# threshold estimation
threshold = Features.estimate_threshold(INPUT_PC, 2 * k)



# normals
normals = Features.compute_normals(INPUT_PC.coordinates, threshold, k)
INPUT_PC.normals = normals

# seeds indices
# masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_COLOMBELLA.txt"
# given_seeds = FileManager.load_points(masterseeds)
# seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

seeds = Int64[]
# outliers
outliers = Features.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

# process
params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

# 2. Detection
hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)

# hyperplane, cluster, all_visited_verts = Detection.get_hyperplane(params)

centroid = Common.centroid(INPUT_PC.coordinates)

V, FV = DrawPlanes(hyperplanes, aabb)

Visualization.VIEW([
    Visualization.points(
        Common.apply_matrix(Common.t(-centroid...), INPUT_PC.coordinates),
        INPUT_PC.rgbs,
    ),
    # Visualization.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
    Visualization.points(
        Common.apply_matrix(
            Common.t(-centroid...),
            INPUT_PC.coordinates[:, outliers],
        );
        color = Visualization.COLORS[2],
    ),
    Visualization.GLGrid(
        Common.apply_matrix(Common.t(-centroid...), V),
        FV,
        Visualization.COLORS[1],
        0.8,
    ),
])

mesh = []
for i = 1:length(hyperplanes)
    push!(
        mesh,
        Visualization.points(
            hyperplanes[i].inliers.coordinates;
            color = Visualization.COLORS[rand(1:12)],
        ),
    )
end

Visualization.VIEW(mesh)
