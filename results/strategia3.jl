using Common
using FileManager
using OrthographicProjection
using AlphaStructures
using Visualization
using Detection

NAME_PROJ = "MURI"
folder = "C:/Users/marte/Documents/GEOWEB/TEST"

function test(folder,NAME_PROJ)
	files = FileManager.searchfile(joinpath(joinpath(folder,NAME_PROJ),"PLANES"),".las")
	out = Array{Lar.Struct,1}()
	for file in files
		h,_ = LasIO.FileIO.load(file)
		if h.records_count > 1000
			@show "sono qui"
			PC = FileManager.las2pointcloud(file)
			points = PC.coordinates
			plane = Plane(points)
			V = Common.apply_matrix(plane.matrix,points)[1:2,:]

			# 2. applica alpha shape con alpha = threshold
			DT = Common.delaunay_triangulation(V)
			filtration = AlphaStructures.alphaFilter(V,DT);
			threshold = Common.estimate_threshold(V,40)
			_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

			# 3. estrai bordo
			EV_boundary = Common.get_boundary_edges(V,FV)
			w,EW = Lar.simplifyCells(V,EV_boundary)

			par = 0.06
			failed = 100
			N = 10
			k = 5

			# threshold estimation
			threshold = 0.06
			# outliers
			outliers = Int64[]
			# process
			params = Initializer(PointCloud(w, zeros(3,size(w,2))),par,threshold,failed,N,k,outliers)
			seeds = Int64[]

			hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)

			try
				V,EV = Common.DrawLines(hyperplanes,0.0)
				V = Common.apply_matrix(Lar.inv(plane.matrix), vcat(V,zeros(size(V,2))'))

				out = push!(out, Lar.Struct([(V, EV)]))
			catch y
			end
		end
	end
	out = Lar.Struct(out)
	W,EW = Lar.struct2lar(out)
	return W, EW
end

W,EW = test(folder,NAME_PROJ)
centroid = Common.centroid(W)

GL.VIEW([
	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),W)')),
	GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),W),EW,GL.COLORS[1],1.0),
])
