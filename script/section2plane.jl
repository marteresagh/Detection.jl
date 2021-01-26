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
	"--output", "-o"
		help = "Output folder"
		required = true
	"--lod"
		help = "Level of detail. If -1, all points are taken"
		arg_type = Int64
		default = -1
	end

	return parse_args(s)
end

function main()
	args = parse_commandline()

	source = args["source"]
	output_folder = args["output"]
	lod = args["lod"]

	PC = Detection.FileManager.source2pc(source, lod)

	# plane description
	direction, centroid = Common.LinearFit(PC.coordinates)
	plane = Detection.Plane(direction,centroid)

	io = open(joinpath(output_folder,"planeCoeff.csv"),"w")
	write(io, "$(plane.a) $(plane.b) $(plane.c) $(plane.d)")
	close(io)
end

@time main()
