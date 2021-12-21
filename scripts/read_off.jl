println("loading packages...")

using ArgParse
using Visualization
using Common

println("packages OK")


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

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Input Potree"
		required = true
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	source = args["source"]
	println("== Parameters ==")
	println("Source  =>  $source")

	flush(stdout)

	V, FV = read_OFF(source)
	FVs = faces2triangles(V, FV)
	Visualization.VIEW(Visualization.GLExplode(V,FVs,1.,1.,1.,99,1.))

end

@time main()
