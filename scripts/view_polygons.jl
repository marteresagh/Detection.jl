println("loading packages...")

using ArgParse
using Visualization
using Common
using FileManager

println("packages OK")


function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"folder"
		help = "polygon folder"
		required = true
	end

	return parse_args(s)
end

"""
"""
function load_connected_components(filename::String)::Common.Cells
	EV = Array{Int64,1}[]
	io = open(filename, "r")
	string_conn_comps = readlines(io)
	close(io)

	conn_comps = [tryparse.(Float64,split(string_conn_comps[i], " ")) for i in 1:length(string_conn_comps)]
	for comp in conn_comps
		for i in 1:(length(comp)-1)
			push!(EV, [comp[i],comp[i+1]])
		end
		push!(EV,[comp[end],comp[1]])
	end
	return EV
end


function main()
	args = parse_commandline()

	folder = args["folder"]
	println("== Parameters ==")
	println("Folder  =>  $folder")

	flush(stdout)

	println("Read data")
	out = Array{Common.Struct,1}()
	for dir in readdir(folder)
		V = FileManager.load_points(joinpath(folder,dir,"points3D.txt"))
		EV = load_connected_components(joinpath(folder,dir,"edges.txt"))
		push!(out, Common.Struct([(V,EV)]))
	end
	out = Common.Struct( out )
	V, EV = Common.struct2lar(out)
	println("View")
	Visualization.VIEW([Visualization.GLGrid(V,EV)])

end

@time main()
