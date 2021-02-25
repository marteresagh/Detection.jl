
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
				V,EV = Common.DrawLines(hyperplanes)
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



function extract_planes(folder,NAME_PROJ)
	volumes = Volume[]
	for (root, dirs, files) in walkdir(joinpath(folder,NAME_PROJ))
		for dir in dirs
			folder_plane = joinpath(root,dir)

			inliers = FileManager.load_points(joinpath(folder_plane,"inliers.txt"))[1:3,:]
			vol = Common.ch_oriented_boundingbox(inliers)
			model = getmodel(vol)

			#OrthographicProjection.segment("C:/Users/marte/Documents/potreeDirectory/pointclouds/MURI", joinpath(joinpath(folder,NAME_PROJ),"$dir.las"), model)
		end
	end
	return true
end

# extract_planes(folder,NAME_PROJ)

function alpha_shapes(folder,NAME_PROJ)
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
			#w, EW = Detection.linearization(w,EW)
			W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))

			out = push!(out, Lar.Struct([(W, EW)]))
		end
	end
	out = Lar.Struct(out)
	W,EW = Lar.struct2lar(out)
	return W, EW
end




function planes_intersection_test(planes, aabb)
	function model_planes(planes::Array{Plane,1}, boxes)
		out = Array{Lar.Struct,1}()
		for i in  1:length(planes)
			plane = planes[i]
			direction = plane.normal
			centroid = plane.centroid
			box = boxes[i]
			V = Common.box_intersects_plane(box,direction,centroid)
			#triangulate vertex projected in plane XY
			points2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
			FV = Common.delaunay_triangulation(points2D)
			EV = Common.get_boundary_edges(points2D,FV)
			cell = (V,sort.(EV),[[1:size(V,2)...]])
			push!(out, Lar.Struct([cell]))
		end
		out = Lar.Struct( out )
		V,EV,FV = Lar.struct2lar(out)
		return V,EV, FV
	end

	V,EV,FV = model_planes(planes,aabb)

	copEV = Lar.coboundary_0(EV::Lar.Cells);
	copFE = Lar.coboundary_1(V, FV::Lar.Cells, EV::Lar.Cells);
	W = convert(Lar.Points, V');

	rV, rcopEV, rcopFE = Lar.Arrangement.spatial_arrangement_1( W,copEV,copFE,false)

	return rV, rcopEV, rcopFE
end


function test(folder,NAME_PROJ)
	files = FileManager.searchfile(joinpath(joinpath(folder,NAME_PROJ),"PLANES"),".las")
	planes = Plane[]
	AABBs = AABB[]
	OBBs = Volume[]
	for file in files
		h,_ = LasIO.FileIO.load(file)
		if h.records_count > 1000
			PC = FileManager.las2pointcloud(file)
			points = PC.coordinates
			aabb = Common.boundingbox(points)
			obb = Common.ch_oriented_boundingbox(points)
			plane = Plane(points)
			push!(planes,plane)
			push!(AABBs,aabb)
			push!(OBBs,obb)
		end
	end
	return planes, AABBs, OBBs
end
