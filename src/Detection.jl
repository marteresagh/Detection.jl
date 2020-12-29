module Detection

    using Common
    using FileManager
    using LightGraphs
    using StatsBase

    # struct
    include("struct.jl")

    # code
    include("Detection/detection.jl")
    include("Detection/main.jl")
    include("Detection/util.jl")
    include("Detection/saves.jl")
    include("Shape/boundary.jl")

    export Initializer, Common, FileManager, LightGraphs
end # module
