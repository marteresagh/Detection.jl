module Detection

    using Common
    using FileManager

    include("struct.jl")

    include("Detection/detection.jl")
    include("Detection/main.jl")
    include("Detection/util.jl")

    export Initializer
end # module
