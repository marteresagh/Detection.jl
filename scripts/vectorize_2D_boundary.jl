println("loading packages... ")

using ArgParse
using Detection
using Common
using AlphaStructures

println("packages OK")

# function boundary_shape

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
	"source"
		help = "Input Potree"
		required = true
	"--projectname", "-p"
		help = "Project name"
		required = true
	"--output", "-o"
		help = "Output folder"
		required = true
	"--par"
		help = "Parameter"
		arg_type = Float64
		default = 0.02
	"--angle"
		help = "Angle"
		arg_type = Float64
		default = pi/8
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 40
	end

	return parse_args(s)
end



function save_boundary(potree::String, folders::Array{String,1}, par::Float64, angle::Float64, k::Int64)

	# function outliers(points, par)
	# 	outliers = Int64[]
	# 	tree = Common.KDTree(points)
	# 	idxs = Common.inrange(tree, points, par)
	# 	for i in 1:length(idxs)
	# 		if length(idxs[i])<3
	# 			push!(outliers,i)
	# 		end
	# 	end
	# 	return outliers
	# end

	n_planes = length(folders)
	Threads.@threads for i in 1:n_planes
		if !isfile(joinpath(folders[i],"execution.probe")) # eventualmente da togliere
			Detection.flushprintln()
			Detection.flushprintln("==========================================================")
			Detection.flushprintln("= $(folders[i]) = $i of $n_planes =")
			Detection.flushprintln("==========================================================")

			file = joinpath(folders[i],"full_inliers.las")
			PC = FileManager.las2pointcloud(file)


			plane = Plane(PC.coordinates)
			V = Common.apply_matrix(plane.matrix,PC.coordinates)[1:2,:]
			# ----------------> da qui coordinate sempre 2D
			#decimazione
			if size(V,2) > 3000000
				V = Features.subsample_poisson_disk(V, 0.05)
				Detection.flushprintln("Decimation: $(size(V,2)) of $(PC.n_points)")
			end

			# out = outliers(T, par) #Common.outliers(PC, collect(1:PC.n_points), 30)
			# V = T[:, setdiff( collect(1:PC.n_points), out)]

			# alpha shape
			if size(V,2) > k
				Detection.flushprintln()
				Detection.flushprint("Alpha shapes....")
				DT = Common.delaunay_triangulation(V)
				filtration = AlphaStructures.alphaFilter(V,DT);
				threshold = Features.estimate_threshold(V,k)
				_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
				Detection.flushprintln("Done")

				# boundary extraction
				Detection.flushprintln()
				Detection.flushprint("Boundary extraction....")
				EV_boundary = Common.get_boundary_edges(V,FV)
				W,EW = Common.simplifyCells(V,EV_boundary)
				model = (W,EW)
				Detection.flushprintln("Done")

				# boundary semplification
				try
					Detection.flushprint("Boundary semplification....")
					V2D, EV = Detection.simplify_model(model; par = par, angle = angle)
					Detection.flushprintln("Done")
					V3D = Common.apply_matrix(Lar.inv(plane.matrix), vcat(V2D,zeros(size(V2D,2))'))

					# save data
					Detection.flushprintln()
					Detection.flushprint("Saves $(length(EV)) edges....")
					if length(EV)>2
						FileManager.save_points_txt(joinpath(folders[i],"boundary_points2D.txt"), V2D)
						FileManager.save_points_txt(joinpath(folders[i],"boundary_points3D.txt"), V3D)
						Detection.save_cycles(joinpath(folders[i],"boundary_edges.txt"), V3D, EV)
						FileManager.successful(true, folders[i])
					end

					Detection.flushprintln("Done")
					Detection.flushprintln()
				catch y
					Detection.flushprintln("NOT FOUND")
				end
			end
		end
	end
end


function main()
	args = parse_commandline()

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	par = args["par"]
	angle = args["angle"]
	k = args["k"]

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("Angle  =>  $angle")

	folders = Detection.get_plane_folders(output_folder, project_name)
	save_boundary(source, folders, par, angle, k)

end

@time main()
