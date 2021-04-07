module Detection

    using Common
    using FileManager
    using LightGraphs
    using DataStructures
    using AlphaStructures

    # struct
    include("struct.jl")

    # code
    # hyperplane detection
    include("Detection/detection.jl")
    include("Detection/cluster.jl")
    include("Detection/util.jl")
    # lines
    include("Detection/Lines/detection.jl")
    include("Detection/Lines/main.jl")
    include("Detection/Lines/saves.jl")
    # planes
    include("Detection/Planes/detection.jl")
    include("Detection/Planes/main.jl")
    include("Detection/Planes/util.jl")
    # flat shape detection
    # include("Shape/tentativo.jl")
    include("Shape/simplify_model.jl")

    export Initializer, Common, FileManager, LightGraphs
end # module
