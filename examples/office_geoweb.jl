using Common
using FileManager
using Detection
using Visualization
using PlyIO

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


println("Read data")
pc = FileManager.source2pc(raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\GEOWEB",3)
M = Common.r(0,0, pi/4)*Common.r(pi/2,0,0)
pc.coordinates = Common.apply_matrix(M, pc.coordinates)
source = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\GEOWEB\segments.ply"
ply_datastructures = PlyIO.load_ply(source)
vertices = ply_datastructures["vertex"]
V = permutedims(hcat(vertices["x"], vertices["y"], vertices["z"]))
V = convert.(Float64,V)
V = Common.apply_matrix(M, V)
normals = permutedims(hcat(vertices["nx"], vertices["ny"], vertices["nz"]))
segment_indices = vertices["segment_index"]

index_planes = unique(segment_indices)

VVs = Vector{Int}[]
for idx in index_planes
    if idx != -1
        push!(VVs, findall(x -> x == idx, segment_indices))
    end
end

planes = Detection.Hyperplane[]
for points in VVs
    pps = V[:, points]
    direction, centroid = Common.LinearFit(pps)
    hyperplane = Detection.Hyperplane(PointCloud(pps), direction, centroid)
    push!(planes, hyperplane)
end

VPlane,EVPlane,FVPlane = DrawPlanes(
    planes;
    box_oriented = false,
)

println("Total planes $(length(VVs))")
println("View")
Visualization.VIEW([
    Visualization.GLGrid(VPlane,FVPlane, Visualization.COLORS[1], 0.2),
    Visualization.points(pc.coordinates,pc.rgbs)
])

Visualization.VIEW([
    [Visualization.points(V[:,VVs[i]]; color = Visualization.COLORS[rand(1:12)], alpha = 0.4) for i in 1:length(VVs)]...,
    Visualization.points(pc.coordinates,pc.rgbs)])
