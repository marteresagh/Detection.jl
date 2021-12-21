using Arrangement
using FileManager
using Common
using Visualization
using Detection

include("remove_faces.jl")
include("lar_model_planes.jl")

V = Common.approxVal(2).(V)

T, ET, ETs, FT, FTs = Arrangement.model_intersection(V, EV, FV)
Visualization.VIEW(Visualization.GLExplode(T, ETs, 1.0, 1.0, 1.0, 99, 1));
Visualization.VIEW(Visualization.GLExplode(T, FTs, 1.0, 1.0, 1.0, 99, 1));
model = (T, ET, ETs, FT, FTs)

faces = copy(FT)
points = copy(T)
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_IDEALE"
remove_faces!(potree, points, faces)

Visualization.VIEW([Visualization.GLGrid(T, faces)])
# Visualization.VIEW(Visualization.GLExplode(W, EWs, 1.0, 1.0, 1.0, 99, 1));
# Visualization.VIEW([
    #Visualization.GLPoints(permutedims(pointcloud), Visualization.COLORS[1]),
    # Visualization.GLExplode(W, FWs, 1.0, 1.0, 1.0, 99, 1)...,
# ]);

edges, triangles, regions = Detection.clustering_faces(T,faces)
Visualization.VIEW([
    Visualization.GLExplode(T, [triangles[regions[i]] for i in 1:length(regions) ], 1.0, 1.0, 1.0, 99, 1)...,
]);
