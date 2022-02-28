using Common
using FileManager
using Visualization


source = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\PotreeRoot"

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
