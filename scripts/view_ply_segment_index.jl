println("loading packages...")

using ArgParse
using Visualization
using Common
using FileManager
using PlyIO

println("packages OK")


function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Input Potree"
		required = true
	end

	return parse_args(s)
end


function main()
	args = parse_commandline()

	source = args["source"]
	println("== Parameters ==")
	println("Source  =>  $source")

	flush(stdout)

	println("Read data")
	ply_datastructures = PlyIO.load_ply(source)
	vertices = ply_datastructures["vertex"]
	V = permutedims(hcat(vertices["x"],vertices["y"],vertices["z"]))
	normals = permutedims(hcat(vertices["nx"],vertices["ny"],vertices["nz"]))
	segment_indices = vertices["segment_index"]

	index_planes = unique(segment_indices)

	VVs = Vector{Int}[]
	for idx in index_planes
		if idx != -1
			push!(VVs, findall(x->x==idx, segment_indices))
		end
	end

	println("Total planes $(length(VVs))")
	println("View")
	Visualization.VIEW([Visualization.points(V[:,VVs[i]]; color = Visualization.COLORS[rand(1:12)]) for i in 1:length(VVs)])

end

@time main()
