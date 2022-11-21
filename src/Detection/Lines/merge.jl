"""
Merge delle linee "simili".
"""
function refine_lines!(lines::Vector{Common.Line})
    n_segments = length(lines)

    theta = pi * 10 / 180
    threshold = 0.10

    merged = true
    while merged
        n_segments = length(lines)
        merged = false
        for i = 1:n_segments
            s1 = lines[i]
            n1 = s1.direction

            for j = i+1:n_segments
                s2 = lines[j]
                n2 = s2.direction

                value_dot = Common.dot(n1, n2)
                if Common.abs(value_dot) > Common.cos(theta)
                    d1 = Common.distance_point2line(s1.startPoint,s1.direction)(s2.startPoint)
                    d2 = Common.distance_point2line(s1.startPoint,s1.direction)(s2.endPoint)
                    d = max(d1,d2)
                    # println(" ============= nuova coppia")
                    # println("S1: $(s1.startPoint),$(s1.endPoint), S2: $(s2.startPoint),$(s2.endPoint) ")

                    s1_aabb = Common.AABB(hcat(s1.startPoint,s1.endPoint))
                    s2_aabb = Common.AABB(hcat(s2.startPoint,s2.endPoint))

                    s1_aabb.x_min-=0.1
                    s1_aabb.x_max+=0.1
                    s1_aabb.y_min-=0.1
                    s1_aabb.y_max+=0.1
                    s2_aabb.x_min-=0.1
                    s2_aabb.x_max+=0.1
                    s2_aabb.y_min-=0.1
                    s2_aabb.y_max+=0.1

                    # println("   BB: $s1_aabb $s2_aabb")
                    # println("   intersezione: $(Common.AABBdetection(s1_aabb,s2_aabb))")
                    # println("   dist max: $d, threshold: $threshold")

                    if Common.AABBdetection(s1_aabb,s2_aabb)
                        if d < threshold
                            merge(lines, i, j)
                            merged = true
                            break
                        end
                    end

                end

            end
            if merged
                break
            end
        end

    end

end

"""
update dell'array di linee.
"""
function merge(lines::Vector{Common.Line}, indx_s1::Int, indx_s2::Int)
    line_1 = lines[indx_s1]
    line_2 = lines[indx_s2]
    points = hcat(line_1.startPoint,line_1.endPoint,line_2.startPoint,line_2.endPoint)

    line_merged = Common.Line(points)

    push!(lines, line_merged)
    deleteat!(lines, [indx_s1, indx_s2])
end
