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



function save_boundary(potree::String, folders::Array{String,1}, hyperplanes::Array{Hyperplane,1}, par::Float64, angle::Float64, k::Int64)
	n_planes = length(folders)
	Threads.@threads for i in 1:n_planes
		Detection.flushprintln()
		Detection.flushprintln("==========================================================")
		Detection.flushprintln("=================== $i of $n_planes ======================")
		Detection.flushprintln("==========================================================")

		# alpha shape of full inliers
		Detection.flushprintln()
		Detection.flushprint("Alpha shapes....")
		file = joinpath(folders[i],"full_inliers.las")
		#######################################
		# se troppi punti si possono decimare #
		#######################################
		function outliers(points, par)
			outliers = Int64[]
			tree = Common.KDTree(points)
			idxs = Common.inrange(tree, points, par)
			for i in 1:length(idxs)
				if length(idxs[i])<3
					push!(outliers,i)
				end
			end
			return outliers
		end

		PC = FileManager.las2pointcloud(file)
		points = PC.coordinates
		plane = Plane(points)
		T = Common.apply_matrix(plane.matrix,points)[1:2,:]

		out = outliers(T, par) #Common.outliers(PC, collect(1:PC.n_points), 30)
		V = T[:, setdiff( collect(1:PC.n_points), out)]

		DT = Common.delaunay_triangulation(V)
		filtration = AlphaStructures.alphaFilter(V,DT);
		threshold = Common.estimate_threshold(V,k)
		_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)
		Detection.flushprintln("Done")

		# boundary extraction
		Detection.flushprintln()
		Detection.flushprint("Boundary extraction....")
		EV_boundary = Common.get_boundary_edges(V,FV)
		w,EW = Lar.simplifyCells(V,EV_boundary)
		W = Common.apply_matrix(Lar.inv(plane.matrix), vcat(w,zeros(size(w,2))'))
		model = (W,EW)
		Detection.flushprintln("Done")

		# boundary semplification
		try
			Detection.flushprint("Boundary semplification....")
			V, EV = Detection.simplify_model(model; par = par, angle = angle)
			Detection.flushprintln("Done")

			# save data
			Detection.flushprintln()
			Detection.flushprint("Saves $(length(EV)) edges....")
			if length(EV)>2
				V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
				FileManager.save_points_txt(joinpath(folders[i],"boundary_points2D.txt"), V2D)
				FileManager.save_points_txt(joinpath(folders[i],"boundary_points3D.txt"), V)
				FileManager.save_connected_components(joinpath(folders[i],"boundary_edges.txt"), V, EV)
				FileManager.successful(true, folders[i])
			end
			Detection.flushprintln("Done")
			Detection.flushprintln()
		catch y
			Detection.flushprintln("NOT FOUND")
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

	folders = get_plane_folders(output_folder, project_name)
	hyperplanes, OBBs = get_hyperplanes(folders)

	save_boundary(source, folders, hyperplanes, par, angle, k)

end

@time main()
