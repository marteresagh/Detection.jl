module Detection

    using Common
    import Common.Points, Common.Point, Common.Cells, Common.LAR
    using FileManager
    using Search
    using Features
    using Printf

    # struct
    include("struct.jl")
    include("keyboard_event.jl")

    # # code
    # # hyperplane detection
    include("Detection/detection.jl")
    include("Detection/cluster.jl")
    # #include("Detection/util.jl")
    #
    # # lines
    include("Detection/Lines/detection.jl")
    include("Detection/Lines/main.jl")
    include("Detection/Lines/new/main.jl")
    include("Detection/Lines/saves.jl")
    # # planes
    include("Detection/Planes/detection.jl")
    include("Detection/Planes/main.jl")
    include("Detection/Planes/new/main.jl")    
    include("Detection/Planes/fileIO.jl")
    include("Detection/Planes/simplify_model.jl")
    include("Detection/Planes/util.jl")
    # # flat shape detection
    # # include("Shape/tentativo.jl")
    #
    # # Arrangement
    include("Arranged_faces/clustering_triangles.jl")
    include("Arranged_faces/merge_planes.jl")
    include("Arranged_faces/read_off.jl")
    include("Arranged_faces/save_ply.jl")

    export Initializer, Common, FileManager, Features
end # module
