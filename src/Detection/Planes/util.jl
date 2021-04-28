
"""
	W,CW = simplifyCells(V,CV)

Find and remove the duplicated vertices and the incorrect cells.
Some vertices may appear two or more times, due to numerical errors
on mapped coordinates. Close vertices are identified, according to the
PRECISION number of significant digits.
"""
function simplifyCells(V,CV)
	PRECISION = 5
	vertDict = Dict{Array{Float64,1}, Int64}(0)
	index = 0
	W = Array{Float64,1}[]
	FW = Array{Int64,1}[]

	for incell in CV
		outcell = Int64[]
		for v in incell
			vert = V[:,v]
			key = map(Common.approxVal(PRECISION), vert)
			if vertDict[key]==0
				index += 1
				vertDict[key] = index
				push!(outcell, index)
				push!(W,key)
			else
				push!(outcell, vertDict[key])
			end
		end
		append!(FW, [[Set(outcell)...]])
	end
	return hcat(W...),FW
end
