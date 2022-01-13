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
            for j in 1:size(inliers,2)
                x = @sprintf("%f", inliers[1,j])
                y = @sprintf("%f", inliers[2,j])
                z = @sprintf("%f", inliers[3,j])
                a = @sprintf("%f", plane.direction[1])
                b = @sprintf("%f", plane.direction[2])
                c = @sprintf("%f", plane.direction[3])
                write(s,"$x $y $z ")
                write(s,"$a $b $c ")
                write(s,"$(segment_index-1)\n")
            end
        end

    end
end
