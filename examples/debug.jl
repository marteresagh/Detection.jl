using Common
using FileManager
using Visualization
using Detection
using AlphaStructures
using Features
Features.DataStructures

function subsample_poisson_disk(points::Common.Points, min_dist=0.05::Float64; step=1.0::Float64 )

	function neighbors_test(s,point,min_dist)
		test = true
		ids_neigh = [[0,0],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1]]
		for i in ids_neigh
			if haskey(voxs,s+i)
				p_neigh = voxs[s+i]
				for pt in p_neigh
					dist = Common.norm(point-pt)
					test = test && dist>min_dist
					if !test
						break
					end
				end
			end
		end
		return test
	end

	npoints = size(points,2)
	voxs = Features.DataStructures.SortedDict{Array{Float64,1},Array{Array{Float64,1},1}}()

	for i in 1:npoints
		point = points[:,i]
		s = floor.(Int,point/step) # compute representative vertex
		# 8 vicini
		if haskey(voxs,s)
			push!(voxs[s],point)
		else
			voxs[s] = [point]
		end
	end

	@show "fatto"

	for (k,s) in voxs
		todel = Int64[]
		for i in 1:length(s)
			point = s[i]
			# 8 vicini
			ids_neigh = [[0,0],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1]]
			interno = true
			for n in ids_neigh
				interno = interno && haskey(voxs,k+n)
				if !interno
					break
				end
			end
			if interno
				if !neighbors_test(k,point,min_dist)
					push!(todel,i)
					# deleteat!(voxs[s],i)
				end
			end
		end
		voxs[k] = voxs[k][setdiff(collect(1:length(s)),todel)]
	end

    a = collect(values(voxs))
	decimation = hcat(collect(Iterators.flatten(a))...)
    return decimation
end

file = "C:\\Users\\marte\\Documents\\GEOWEB\\TEST\\LA_CONTEA_LOD3\\plane_63780523433408\\full_inliers.las"
PC = FileManager.las2pointcloud(file)

V = rand(2,10000)
plane = Plane(PC.coordinates)
V = Common.apply_matrix(plane.matrix,PC.coordinates)[1:2,:]
#decimazione
V = Features.subsample_poisson_disk(V, 0.02)
Visualization.VIEW([
	Visualization.points(V)
])
k = 40
DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = Features.estimate_threshold(V,k)
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

Visualization.VIEW([
	Visualization.GLGrid(W,EW)
])

EV_boundary = Common.get_boundary_edges(V,FV)
W,EW = Detection.simplifyCells(V,EV_boundary)
model = (W,EW)

# open("model.txt", "w") do f
# 	write(f,"V = $W\n\n")
# 	write(f,"EV = $EW\n\n")
# 	close(f)
# end
V2D, EV = Detection.simplify_model(model; par = 0.02, angle = pi/8)


Visualization.VIEW([
	Visualization.GLGrid(V2D, EV)
])


GL.VIEW([
	[GL.GLGrid(w,EW[comp], GL.COLORS[rand(1:12)]) for comp in conn_comps]...
])

model = (w,EW)
w_,ew_ = Detection.simplify_model(model; par = 0.01, angle = pi/8)
GL.VIEW([
	GL.GLGrid(w_,ew_)
])

graph = Common.model2graph_edge2edge(w_,ew_)
# conn_comps = LightGraphs.connected_components(graph)
conn_comps = cycle(w_,ew_)
GL.VIEW([
	GL.GLGrid(w_,ew_[conn_comps[3]])
])

w3D_ = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w_,zeros(size(w_,2))'))
