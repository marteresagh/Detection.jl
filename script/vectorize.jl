println("loading packages... ")

using ArgParse
using Detection
using OrthographicProjection

println("packages OK")

"""
generate input point cloud
"""
function source2pc(source::String, plane::Detection.Plane, thickness::Float64)

	if isdir(source) # se source è un potree
		Detection.flushprintln("Potree struct")
		cloud_metadata = Detection.CloudMetadata(source)
		bbin = cloud_metadata.tightBoundingBox
		model = OrthographicProjection.Common.plane2model(plane,thickness,bbin)
		aabb = Detection.Common.boundingbox(model[1])
		mainHeader = Detection.FileManager.newHeader(aabb,"EXTRACTION",Detection.SIZE_DATARECORD)

		params = OrthographicProjection.ParametersExtraction("slice.las",
															[source],
															Matrix{Float64}(Detection.Lar.I,3,3),
															model,
															-Inf,
															Inf,
															mainHeader
															)

		OrthographicProjection.segment_and_save(params)

		return Detection.FileManager.las2pointcloud(params.outputfile)

	elseif isfile(source) # se source è un file
		#Detection.flushprintln("Single file")
		bbin = Detection.FileManager.las2aabb(source)
		model = OrthographicProjection.Common.plane2model(plane,thickness,bbin)
		PC = Detection.FileManager.las2pointcloud(source)
		if Detection.Common.modelsdetection(model, bbin) == 2 # full model
			Detection.flushprintln("full model")
			return PC
		else
			Detection.flushprintln("slice")
			tokeep = Detection.Common.inmodel(model).([PC.coordinates[:,i] for i in 1:PC.n_points])
			return Detection.PointCloud(PC.coordinates[:,tokeep],PC.rgbs[:,tokeep])
		end

	end

end



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
		required = true
	"--threshold"
		help = "Level of detail. If -1, all points are taken"
		arg_type = Float64
		required = true
	"--failed"
		help = "number of failed before exit"
		arg_type = Int64
		default = 100
	"--validity"
		help = "number of points in a line"
		arg_type = Int64
		default = 5
	"--k"
		help = "number of neighbors"
		arg_type = Int64
		default = 10
	"--plane"
		help = "a, b, c, d parameters described the plane"
		arg_type = String
		required = true
	"--thickness"
		help = "Sections thickness"
		arg_type = Float64
		required = true
	end

	return parse_args(s)
end

function main()
	args = parse_commandline()

	# Detection.flushprintln("== params ==")
	# for (arg,val) in args
	#     Detection.flushprintln("$arg  =>  $val")
	# end

	source = args["source"]
	project_name = args["projectname"]
	output_folder = args["output"]
	par = args["par"]
	failed = args["failed"]
	N = args["validity"]
	k = args["k"]
	threshold = args["threshold"]
	plane = args["plane"]
	thickness = args["thickness"]

	b = tryparse.(Float64,split(plane, " "))
	@assert length(b) == 4 "$plane: Please described the plane in Hessian normal form"
	plane = Detection.Plane(b[1],b[2],b[3],b[4])
	affine_matrix = plane.matrix

	Detection.flushprintln("== Parameters ==")
	Detection.flushprintln("Source  =>  $source")
	Detection.flushprintln("Output folder  =>  $output_folder")
	Detection.flushprintln("Project name  =>  $project_name")
	Detection.flushprintln("Threshold =>  $threshold")
	Detection.flushprintln("Parameter  =>  $par")
	Detection.flushprintln("N. of failed  =>  $failed")
	Detection.flushprintln("N. of points on line  =>  $N")
	Detection.flushprintln("N. of k-nn  =>  $k")
	Detection.flushprintln("Affine matrix =>  $affine_matrix")

	PC = source2pc(source::String, plane::Detection.Plane, thickness::Float64)
	#che trheshold gli passo?? glielo chiedo all'utente??
	Detection.pc2vectorize(output_folder, project_name, PC, par, threshold, failed, N, k, affine_matrix)
end

@time main()
