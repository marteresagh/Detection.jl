using Detection
using FileManager
using Common

using DataStructures

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/Sezione_LACONTEA"
cloud_metadata = CloudMetadata(source)
trie = FileManager.potree2trie(source)

TRIE = Trie()

TRIE["c"] = 1
TRIE["ci"] = 2
TRIE["cia"] = 3
TRIE["ciao"] = 4
TRIE["ca"] = 5
TRIE["cas"] = 6
TRIE["casa"] = 7
TRIE["casal"] = 8
TRIE["casale"] = 9
TRIE["casalet"] = 10
TRIE["casalett"] = 11
TRIE["casaletto"] = 12
TRIE["can"] = 13
TRIE["cane"] = 14


function truncate_trie(trie, level, data, l = 0,all_prev = true)
	if all_prev
		if l<=level
			push!(data,trie.value)
			for key in collect(keys(trie.children))
				truncate_trie(trie.children[key],level,data,l+1,all_prev)
			end
		end
	else
		if l==level
			push!(data,trie.value)
		end
		for key in collect(keys(trie.children))
			truncate_trie(trie.children[key],level,data,l+1,all_prev)
		end
	end
	return data
end

data = truncate_trie(TRIE.children['c'],3,[],0,false)

length(TRIE.children)
