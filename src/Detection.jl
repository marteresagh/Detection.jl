module Detection

    using Common
    using FileManager

    include("struct.jl")

    include("Detection/general.jl")
    include("Detection/init.jl")
    include("Detection/util.jl")
end # module
