function PlaneDetection(
	source::String,
	output::String,
	LOD::Int64,
	par::Float64,
	fileseedpoints::String,
	failed::Int64,
	maxnumplanetofind::Int64)

	params = PointClouds.init(source::String,
	output::String,
	LOD::Int64,
	par::Float64,
	fileseedpoints::String,
	failed::Int64,
	maxnumplanetofind::Int64)

	thres = 2*params.cloudMetadata.spacing/2^params.LOD
	if params.rnd
		planes = PlanesDetectionRandom(params.pointcloud, params.par, thres, params.failed)
	else
		planes = PlaneDataset[]
		for data in params.seedPoints
			plane = PlaneDetectionFromGivenPoints(params.pointcloud, data, params.par, thres)
			push!(planes,plane)
		end
	end

	savePlanesDataset(planes)

end



function init(
	source::String,
	output::String,
	LOD::Int64,
	par::Float64,
	fileseedpoints::Union{Nothing,String},
	failed::Int64,
	maxnumplanetofind::Int64)


	#input
	allfile = PointClouds.filelevel(source,LOD,false)
	cloudMetadata = PointClouds.cloud_metadata(source)
	pointcloud,_,rgb = PointClouds.loadlas(allfile...)

	#output
	if !isdir(output)
		mkdir(output)
	end

	#seedpoints
	rnd = true
	seedPoints = Lar.Points
	if !isnothing(fileseedpoints)
		rnd = false
		seedPoints = seedPointsFromFile(fileseedpoints)
	end


	return PlaneDetectionParams(
	pointcloud,
	cloudMetadata,
	UInt16(LOD),
	par,
	output,
	rnd,
	seedPoints,
	random,
	UInt16(failed)
	)
end
