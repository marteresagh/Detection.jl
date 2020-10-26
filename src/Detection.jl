module Detection

    using Common
    using FileManager
    using Statistics

    include("struct.jl")

    include("Detection/detection.jl")
    include("Detection/init.jl")
    include("Detection/util.jl")
end # module
