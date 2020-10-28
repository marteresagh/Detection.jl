module Detection

    using Common
    using FileManager

    include("struct.jl")

    include("Detection/detection.jl")
    include("Detection/init.jl")
    include("Detection/util.jl")

    export initParams
end # module
