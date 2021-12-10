function save_plane_segments_in_ply(hyperplanes, filename)

    total = sum(x->x.inliers.n_points,hyperplanes)
    open(filename, "w") do s
        # write header
        write(s, "ply\n")
        write(s, "format ascii 1.0\n")
        write(s, "comment saved by detection module\n")
        write(s, "element vertex $total\n")
        write(s, "property float x\n")
        write(s, "property float y\n")
        write(s, "property float z\n")
        write(s, "property float nx\n")
        write(s, "property float ny\n")
        write(s, "property float nz\n")
        write(s, "property int segment_index\n")
        write(s, "end_header\n")

        for segment_index in 1:length(hyperplanes)
            plane = hyperplanes[segment_index]
            inliers = plane.inliers.coordinates
            @show plane.inliers.n_points
            V_plane,_ = Common.remove_double_verts(inliers)
            @show size(V_plane,2)
            for j in 1:size(V_plane,2)
                write(s,"$(V_plane[1,j]) $(V_plane[2,j]) $(V_plane[3,j]) ")
                write(s,"$(plane.direction[1]) $(plane.direction[2]) $(plane.direction[3]) ")
                write(s,"$segment_index\n")
            end
        end

    end
end
