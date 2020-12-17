module Detection

    using Common
    using FileManager

    # struct
    include("struct.jl")

    # code
    include("Detection/detection.jl")
    include("Detection/main.jl")
    include("Detection/util.jl")
    include("Detection/saves.jl")

    export Initializer, Common, FileManager
end # module
