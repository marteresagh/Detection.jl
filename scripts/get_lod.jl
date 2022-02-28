println("loading packages...")

using ArgParse
using FileManager
using FileManager.LasIO

println("packages OK")

# function boundary_shape

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Input Potree"
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

	lod = -1

	cloudjs = FileManager.CloudMetadata(source)
	count = cloudjs.points

	if count > 1_000_000
		trie = FileManager.potree2trie(source)
		max_depth = FileManager.max_depth(trie)
		depth = max_depth - 1
		while count > 2_000_000
			len = Float64[]
			all_files = FileManager.get_files(trie, depth)
			for file in all_files
				header = nothing
				open(file,"r") do s
				  LasIO.skiplasf(s)
				  header = read(s, LasIO.LasHeader)
				end
				push!(len, header.records_count)
			end
			count =  sum(len)
			println("count: $count")
			depth -= 1
		end
		lod = depth
	end

	s = open(joinpath(output_folder,"lod.txt"), "w")
	write(s,"{lod: $lod}")
	close(s)

end

@time main()
