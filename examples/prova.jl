using Common
using Visualization

V = [
    5.29999 5.0 5.0 5.299989999999999 5.29999 5.0 5.0 5.299989999999999
    5.51364 5.51282 5.99621 5.99597 5.51364 5.51282 5.99621 5.99597
    2.79998 2.79998 2.79998 2.79998 2.59998 2.59998 2.59998 2.59998
]

EV = [
    [4, 1],
    [1, 2],
    [2, 3],
    [3, 4],
    [1, 5],
    [2, 6],
    [3, 7],
    [4, 8],
    [8, 5],
    [5, 6],
    [6, 7],
    [7, 8],
]

FV = [
    [1, 2, 3, 4],
    [4, 1, 5, 8],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 4, 8, 7],
    [5, 6, 7, 8],
]

point = [5.0, 5.915, 2.2]

Common.point_in_polyhedron(point, V, EV, FV)


function spaceindex(point3d::Array{Float64,1})::Function
    function spaceindex0(model::Common.LAR)::Array{Int,1}
        V, CV = copy(model[1]), copy(model[2])
        V = [V point3d]
        dim, idx = size(V)
        @show dim, idx
        push!(CV, [idx, idx, idx])
        cellpoints = [V[:, CV[k]]::Common.Points for k = 1:length(CV)]
        @show CV
        #----------------------------------------------------------
        bboxes = [hcat(Common.pboundingbox(cell)...) for cell in cellpoints]

        xboxdict = Common.coordintervals(1, bboxes)
        yboxdict = Common.coordintervals(2, bboxes)

        # xs,ys are IntervalTree type
        xs = Common.IntervalTrees.IntervalMap{Float64,Array}()
        for (key, boxset) in xboxdict
            xs[tuple(key...)] = boxset
        end
        ys = Common.IntervalTrees.IntervalMap{Float64,Array}()
        for (key, boxset) in yboxdict
            ys[tuple(key...)] = boxset
        end


        xcovers = Common.boxcovering(bboxes, 1, xs)
        ycovers = Common.boxcovering(bboxes, 2, ys)
        @show xcovers

        @show ycovers

        covers = [intersect(pair...) for pair in zip(xcovers, ycovers)]
        @show covers

        # add new code part

        # remove each cell from its cover
        pointcover = setdiff(covers[end], length(CV))
        return pointcover
    end
    return spaceindex0
end


faces = spaceindex(point)((V, FV))

testinternalpoint(V, EV, FV)(point)

Visualization.VIEW([
    Visualization.GLGrid(V, [FV[6]]),
    Visualization.GLGrid([point [5.0, 5.915, 4.2]], [[1, 2]]),
    Visualization.points(point),
])



function testinternalpoint(V::Common.Points, EV::Common.Cells, FV::Common.Cells)
    copEV = Common.lar2cop(EV)
    copFV = Common.lar2cop(FV)
    copFE = copFV * copEV'
    I, J, Val = Common.findnz(copFE)
    triple = zip([(i, j, 1) for (i, j, v) in zip(I, J, Val) if v == 2]...)
    I, J, Val = map(collect, triple)
    Val = convert(Array{Int8,1}, Val)
    copFE = Common.sparse(I, J, Val)

    function testinternalpoint0(testpoint)
        intersectedfaces = Int64[]
        # spatial index for possible intersections with ray
        faces = Common.spaceindex(testpoint)((V, FV))
        depot = []
        # face in faces :  indices of faces of possible intersection with ray
        @show faces
        for face in faces
            value = Common.rayintersection(testpoint)(V, FV, face)
            if typeof(value) == Array{Float64,1}
                push!(depot, (face, value))
            end
        end
        # actual containment test of ray point in faces within depot
        for (face, point3d) in depot
            vs, edges, point2d = Common.planemap(V, copEV, copFE, face)(point3d)
            classify = Common.pointInPolygonClassification(vs, edges)
            inOut = classify(point2d)
            if inOut != "p_out"
                push!(intersectedfaces, face)
            end
        end
        return intersectedfaces
    end
    return testinternalpoint0
end



# using PyCall
# function larModelProduct(modelOne, modelTwo)
#     (V, cells1) = modelOne
#     (W, cells2) = modelTwo
#
#     vertices = DataStructures.OrderedDict()
#     k = 1
#     for j = 1:size(V, 2)
#         v = V[:, j]
#         for i = 1:size(W, 2)
#             w = W[:, i]
#             id = [v; w]
#             if haskey(vertices, id) == false
#                 vertices[id] = k
#                 k = k + 1
#             end
#         end
#     end
#
#     cells = []
#     for c1 in cells1
#         for c2 in cells2
#             cell = []
#             for vc in c1
#                 for wc in c2
#                     push!(cell, vertices[[V[:, vc]; W[:, wc]]])
#                 end
#             end
#             push!(cells, cell)
#         end
#     end
#
#     vertexmodel = []
#     for v in keys(vertices)
#         push!(vertexmodel, v)
#     end
#     verts = hcat(vertexmodel...)
#     cells = [[v for v in cell] for cell in cells]
#     return (verts, cells)
# end
#
#
# function extrude(points, face, size_extrusion)
#     plane = Common.Plane(points)
#     V2D = Common.apply_matrix(plane.matrix, points)[1:2, :]
#     tri = Common.delaunay_triangulation(V2D)
#     edges = Common.get_boundary_edges(points, tri)
#     modelOne = V2D, edges
#
#     interval = [-size_extrusion / 2 size_extrusion / 2]
#     edge = [[1, 2]]
#     modelTwo = interval, edge
#
#     points_extruded, cells = larModelProduct(modelOne, modelTwo)
#     points_final =
#         Common.apply_matrix(Common.inv(plane.matrix), points_extruded)
#     return points_final, cells
# end
