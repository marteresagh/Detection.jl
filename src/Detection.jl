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
    # flat shape detection
    include("Shape/definitivo.jl")

    export Initializer, Common, FileManager, LightGraphs
end # module
