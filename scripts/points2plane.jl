println("loading packages... ")

using ArgParse
using Common
using FileManager

println("packages OK")

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "A text file with points list"
		required = true
	"--output", "-o"
		help = "Output folder"
		required = true
	end

	return parse_args(s)
end

function main()
	args = parse_commandline()

	source = args["source"]
	output_folder = args["output"]

	points = FileManager.load_points(source)

	# plane description
	try
		direction, centroid = Common.Fit_Plane(points)
		plane = Common.Plane(direction,centroid)

		io = open(joinpath(output_folder,"planeCoeff.csv"),"w")
		write(io, "$(plane.a) $(plane.b) $(plane.c) $(plane.d)")
		close(io)

		FileManager.successful(true, output_folder)
	catch y

	end
end

@time main()
