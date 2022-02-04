using Common
using Detection
using Visualization
using FileManager

n = 50000
V = vcat(10*rand(n)', 10*rand(n)')

hyperplanes = Detection.Hyperplane[]

pavimento_box1 = Common.add_zeta_coordinates(V,0.)

plane = Plane(pavimento_box1)
hyp = Detection.Hyperplane(PointCloud(pavimento_box1,zeros(3,size(pavimento_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

soffitto_box1 = Common.add_zeta_coordinates(V,10.)

plane = Plane(soffitto_box1)
hyp = Detection.Hyperplane(PointCloud(soffitto_box1,zeros(3,size(soffitto_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)


V = vcat(10*rand(n)', 10*rand(n)')
lato_box1 = Common.add_zeta_coordinates(V,0.)
lato1_box1 = Common.apply_matrix(Common.r(pi/2,0,0), lato_box1)

plane = Plane(lato1_box1)
hyp = Detection.Hyperplane(PointCloud(lato1_box1,zeros(3,size(lato1_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato2_box1 = Common.apply_matrix(Common.r(0,0,pi/2), lato1_box1)

plane = Plane(lato2_box1)
hyp = Detection.Hyperplane(PointCloud(lato2_box1,zeros(3,size(lato2_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato3_box1 = Common.apply_matrix(Common.t(0,10,0), lato1_box1)

plane = Plane(lato3_box1)
hyp = Detection.Hyperplane(PointCloud(lato3_box1,zeros(3,size(lato3_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato4_box1 = Common.apply_matrix(Common.t(10,0,0), lato2_box1)

plane = Plane(lato4_box1)
hyp = Detection.Hyperplane(PointCloud(lato4_box1,zeros(3,size(lato4_box1,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

box1 = hcat(pavimento_box1,soffitto_box1,lato1_box1,lato2_box1,lato3_box1,lato4_box1)
Visualization.VIEW([
    Visualization.axis_helper()
    Visualization.points(pavimento_box1)
    Visualization.points(soffitto_box1)
    Visualization.points(lato1_box1; color = Visualization.COLORS[1], alpha = 0.5)
    Visualization.points(lato2_box1; color = Visualization.COLORS[2], alpha = 0.5)
    Visualization.points(lato3_box1; color = Visualization.COLORS[3], alpha = 0.5)
    Visualization.points(lato4_box1; color = Visualization.COLORS[4], alpha = 0.5)
])

V = vcat(5*rand(n)', 5*rand(n)')
pavimento_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.add_zeta_coordinates(V,0.))

plane = Plane(pavimento_box2)
hyp = Detection.Hyperplane(PointCloud(pavimento_box2,zeros(3,size(pavimento_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

soffitto_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.add_zeta_coordinates(V,3.))

plane = Plane(soffitto_box2)
hyp = Detection.Hyperplane(PointCloud(soffitto_box2,zeros(3,size(soffitto_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

V = vcat(5*rand(n)', 3*rand(n)')
lato_box2 = Common.add_zeta_coordinates(V,0.)
lato1_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.apply_matrix(Common.r(pi/2,0,0), lato_box2))

plane = Plane(lato1_box2)
hyp = Detection.Hyperplane(PointCloud(lato1_box2,zeros(3,size(lato1_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato2_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.apply_matrix(Common.r(0,0,pi/2)*Common.r(pi/2,0,0), lato_box2))

plane = Plane(lato2_box2)
hyp = Detection.Hyperplane(PointCloud(lato2_box2,zeros(3,size(lato2_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato3_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.apply_matrix(Common.t(0,5,0)*Common.r(pi/2,0,0), lato_box2))

plane = Plane(lato3_box2)
hyp = Detection.Hyperplane(PointCloud(lato3_box2,zeros(3,size(lato3_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

lato4_box2 = Common.apply_matrix(Common.t(3,1,2)*Common.r(0,0, pi/8),Common.apply_matrix(Common.t(5,0,0)*Common.r(0,0,pi/2)*Common.r(pi/2,0,0), lato_box2))

plane = Plane(lato4_box2)
hyp = Detection.Hyperplane(PointCloud(lato4_box2,zeros(3,size(lato4_box2,2))),plane.normal,plane.centroid)
push!(hyperplanes, hyp)

box2 = hcat(pavimento_box2,soffitto_box2,lato1_box2,lato2_box2,lato3_box2,lato4_box2)
Visualization.VIEW([
    Visualization.axis_helper()
    Visualization.points(pavimento_box2)
    Visualization.points(soffitto_box2)
    Visualization.points(lato1_box2; color = Visualization.COLORS[1], alpha = 0.5)
    Visualization.points(lato2_box2; color = Visualization.COLORS[2], alpha = 0.5)
    Visualization.points(lato3_box2; color = Visualization.COLORS[3], alpha = 0.5)
    Visualization.points(lato4_box2; color = Visualization.COLORS[4], alpha = 0.5)
])

box3 = hcat(box1, box2)
Visualization.VIEW([
    Visualization.axis_helper()
    Visualization.points(box3)
])

PC = PointCloud(box3, FileManager.LasIO.FixedPointNumbers.N0f16.(ones(3,size(box3,2))))

Visualization.VIEW([
    Visualization.axis_helper()
    Visualization.points(PC.coordinates,PC.rgbs )
])

using FileManager

FileManager.save_pointcloud(raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BASIC\pc.las",PC,"test")

Detection.save_plane_segments_in_ply(hyperplanes, raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\BASIC\vect3D\SEGMENTS\segments.ply")
