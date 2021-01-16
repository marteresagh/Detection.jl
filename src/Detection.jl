module Detection

    using Common
    using FileManager
    using LightGraphs

    # struct
    include("struct.jl")

    # code
    # hyperplane detection
    include("Detection/detection.jl")
    include("Detection/cluster.jl")
    include("Detection/main.jl")
    include("Detection/util.jl")
    include("Detection/saves.jl")
    # flat shape detection
    include("Shape/definitivo.jl")

    export Initializer, Common, FileManager, LightGraphs
end # module
