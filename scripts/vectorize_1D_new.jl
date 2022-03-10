println("loading packages... ")

using ArgParse
using Detection

println("packages OK")

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "source"
        help = "Potree directory"
        required = true
        "--transformation", "-t"
        help = "Text file of affine matrix by row"
        arg_type = String
        required = true
        "--projectname", "-p"
        help = "Project name"
        required = true
        "--output", "-o"
        help = "Output folder"
        required = true
        "--par"
        help = "Distance to line"
        arg_type = Float64
        required = true
        "--lod"
        help = "Level of detail. If -1, all points are taken"
        arg_type = Int64
        default = -1
        "--failed"
        help = "Number of failed before exit"
        arg_type = Int64
        default = 100
        "--validity"
        help = "Minimum number of inliers"
        arg_type = Int64
        default = 5
        "--k"
        help = "Number of neighbors"
        arg_type = Int64
        default = 30
        "--masterseeds", "-s"
        help = "A text file with seeds list"
        arg_type = String
    end

    return parse_args(s)
end

function main()
    args = parse_commandline()

    source = args["source"]
    transformation = args["transformation"]
    project_name = args["projectname"]
    output_folder = args["output"]
    par = args["par"]
    failed = args["failed"]
    N = args["validity"]
    k = args["k"]
    lod = args["lod"]
    masterseeds = args["masterseeds"]

    # plane description

    PC = FileManager.source2pc(source, lod)
    LINES = readlines(transformation)
    affine_matrix = [tryparse.(Float64,split(LINES[i], " ")) for i in 1:length(LINES)]
    affine_matrix = vcat(affine_matrix'...)

    println("== Parameters ==")
    println("Source  =>  $source")
    println("User coordinates system  =>  $affine_matrix")
    println("Output folder  =>  $output_folder")
    println("Project name  =>  $project_name")
    println("Parameter  =>  $par")
    println("Seeds =>  $(args["masterseeds"])")
    println("N. of failed  =>  $failed")
    println("N. of inliers  =>  $N")
    println("N. of k-nn  =>  $k")
    println("Level of detail  =>  $lod")

    flush(stdout)

    Detection.pc2lines(
        output_folder,
        project_name,
        PC,
        par,
        failed,
        N,
        k,
        affine_matrix;
        masterseeds = masterseeds,
    )
end

@time main()
