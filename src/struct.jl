struct Parameters
	pointcloud::PointCloud
	cloudMetadata::PointClouds.CloudMetadata
	LOD::UInt16
	par::Float64,
	output::String
	rnd::Bool
	seedPoints::Lar.Points
	random::Bool
	failed::UInt16
end
