using Visualization
using Common

function read_OFF(filename::String)
  V = nothing
  FV = nothing
  open(filename) do f
    firstline = readline(f)
    if firstline != "OFF"
      throw(ErrorException("Expected \"OFF\" header, got \"$firstline\""))
    end
    n_vertices = nothing
    n_faces = nothing
    n_edges = nothing
    # read header
    while true
      line = readline(f)
      if line == "" && eof(f)
        throw(ErrorException("Unexpected end of file reading off header"))
      elseif startswith(line, "#")

      else
        tokens = split(line)
        n_vertices = parse(Int, tokens[1])
        n_faces = parse(Int, tokens[2])
        n_edges = parse(Int, tokens[3])
        break
      end
    end

    if n_vertices != 0
      V = zeros(3, n_vertices)

      while true
        line = readline(f)
        if line == "" && eof(f)
          throw(ErrorException("Unexpected end of file reading off header"))
        elseif startswith(line, "#")

        elseif line == ""

        else
          points = split(line)
          x = parse(Float64, points[1])
          y = parse(Float64, points[2])
          z = parse(Float64, points[3])
          V[:, 1] = [x, y, z]
          for i = 2:n_vertices
            line = readline(f)
            points = split(line)
            x = parse(Float64, points[1])
            y = parse(Float64, points[2])
            z = parse(Float64, points[3])
            V[:, i] = [x, y, z]
          end
          break
        end
      end

      FV = Vector{Int}[]
      while true
        line = readline(f)
        if line == "" && eof(f)
          throw(ErrorException("Unexpected end of file reading off header"))
        elseif startswith(line, "#")

        elseif line == ""

        else
          face_info = split(line)

          n_verts_face = parse(Int, face_info[1])
          face = Int[]
          for i = 2:n_verts_face+1
            push!(face, parse(Int, face_info[i]) + 1)
          end
          push!(FV, face)

          for i = 2:n_faces
            line = readline(f)
            face_info = split(line)

            n_verts_face = parse(Int, face_info[1])
            face = Int[]
            for i = 2:n_verts_face+1
              push!(face, parse(Int, face_info[i]) + 1)
            end
            push!(FV, face)
          end
          break
        end
      end
    end


  end
  return V, FV
end

filename = raw"C:\dev\CGAL-5.3\examples\Polygonal_surface_reconstruction\build\Debug\data\building_result-0.05.off"
V, FV = read_OFF(filename::String)

Visualization.VIEW([Visualization.GLPolyhedron(V, FV)])

function faces2triangles(V, FV)
  FVs = Vector{Vector{Int64}}[]
  for face in FV

    points_face = V[:, face]

    plane = Common.Plane(points_face)

    point_z_zero = Common.apply_matrix(plane.matrix, points_face)[1:2, :]

    triangle = Common.delaunay_triangulation(point_z_zero)
    push!(FVs, map(x -> face[x], triangle))
  end

  return FVs
end

FVs = faces2triangles(V, FV)

Visualization.VIEW(Visualization.GLExplode(V,FVs,1.,1.,1.,99,1.))
# FTs = Vector{Vector{Int64}}[]
#
#     for i = 1:length(ETs)
#         ET = ETs[i]
#         idxs_verts = union(ET...)
#         points_face = V[:, idxs_verts]
#         EV_mapped = [
#             map(x -> findall(j -> j == x, idxs_verts)[1], ET[i])
#             for i = 1:length(ET)
#         ]
# 
#
#         baricentro = get_centroid(points_face)
#         normal = Common.cross(
#             baricentro - points_face[:, 1],
#             points_face[:, 2] - points_face[:, 1],
#         )
#
#         centroid = points_face[:, 1]
#
#         M = affine_transformation(normal, centroid)
#
#
#         point_z_zero = Common.apply_matrix(M, points_face)[1:2, :]
#
#         FT = triangulate2d(point_z_zero, EV_mapped)
#         push!(FTs, map(x -> idxs_verts[x], FT))
#     end
