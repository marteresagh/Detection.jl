using Common
using Visualization
# using PyCall
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

V = [3. 9. 10. 6. 1.;
     2. 1.  7. 9. 7.;
     0. 0. 0. 0. 0.]
#
# V = Common.apply_matrix(Common.r(pi/7,0,0)*Common.r(0,pi/5,0), V)
EV = [[1,2],[2,3],[3,4],[4,5],[5,1]]

FV = [[1,2,3,4,5]]


# mi servono V,EV,FV
# FV = planar_face
function extrude(V, EV, FV, size_extrusion)
    dim, n = size(V)
    if dim == 3
        plane = Common.Plane(V)
        V2D = Common.apply_matrix(plane.matrix, V)[1:2, :]
    else
        V2D = copy(V)
    end

    new_points = hcat(Common.add_zeta_coordinates(V2D,-size_extrusion/2),Common.add_zeta_coordinates(V2D,size_extrusion/2))

    if dim == 3
        V_extruded =
            Common.apply_matrix(Common.inv(plane.matrix), new_points)
    else
        V_extruded = copy(new_points)
    end

    EV_extruded = [EV..., [[i,i+n] for i in 1:n]..., [map(x->x+n,edge) for edge in EV]...]

    FV_extruded = [FV..., [[edge[1],edge[2],edge[2]+n,edge[1]+n] for edge in EV]..., [map(x->x+n,face) for face in FV]...]

    return V_extruded, EV_extruded, FV_extruded
end

model = extrude(V, EV, FV, 1.5)
point = [6,5,0.]
test = Common.point_in_polyhedron(point, model[1], model[2], model[3])
Visualization.VIEW([
        Visualization.GLGrid(model[1],model[2]),
        Visualization.points(point),
        Visualization.axis_helper()...
    ])



using BenchmarkTools

@btime point_in_poly(model[1], point)

@btime Common.point_in_polyhedron(point, model[1], model[2], model[3])


V = [0.9997932850296527 0.9997962850296509 4.999999285029654 4.999999285029651 0.9997947149703492 0.9997977149703474 5.0000007149703505 5.000000714970348; -0.17997319985256338 -0.17997460013927627 -0.18000319985250735 -0.1800018001393333 0.020026800139276782 0.020025399852563894 0.019996800139332818 0.019998199852506837; 2.9535005520897606 2.6998705520897626 2.699970552089761 2.953500552089763 2.9534994479102363 2.6998694479102383 2.6999694479102367 2.9534994479102386]
EV = [[4, 2], [4, 1], [1, 3], [3, 2], [1, 5], [2, 6], [3, 7], [4, 8], [8, 6], [8, 5], [5, 7], [7, 6]]
FV = [[4, 1, 3, 2], [4, 2, 6, 8], [4, 1, 5, 8], [1, 3, 7, 5], [3, 2, 6, 7], [8, 5, 7, 6]]

point = [4.934, 0.02, 2.7]

Common.point_in_polyhedron(point, V,EV,FV)

Visualization.VIEW([
        Visualization.GLGrid(V,EV),
        Visualization.points(point),
        Visualization.axis_helper()...
    ])

#### ricontrollare l'estrusione TODO
