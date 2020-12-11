mutable struct Initializer
	PC::PointCloud
	par::Float64
	threshold::Float64
	failed::Int64
	N::Int64
	k::Int64
	outliers::Array{Int64,1}
	visited::Array{Int64,1}
	current_inds::Array{Int64,1}
	fitted::Array{Int64,1}

	Initializer(PC, par, threshold, failed, N, k, outliers, current_inds) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), current_inds,Int64[])
	Initializer(PC, par, threshold, failed, N, k, outliers) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), collect(1:PC.n_points),Int64[])
	Initializer(PC, par, threshold, failed, N, k) = new(PC, par, threshold, failed, N, k, Int64[], Int64[], collect(1:PC.n_points),Int64[])
end


struct VectDirs
	output_folder::String
	POINTCLOUDS::String
	FULL::String
	PARTITIONS::String
	DXF::String
	RAW::String

	function VectDirs(folder::String, project_name::String)
		output_folder = FileManager.mkdir_project(folder,project_name)
		POINTCLOUDS = joinpath(output_folder,"POINTCLOUDS")
		mkdir(POINTCLOUDS)
		FULL =  joinpath(POINTCLOUDS,"FULL")
		mkdir(FULL)
		PARTITIONS = joinpath(POINTCLOUDS,"PARTITIONS")
		mkdir(PARTITIONS)
		DXF = joinpath(output_folder,"DXF")
		mkdir(DXF)
		RAW = joinpath(DXF,"RAW")
		mkdir(RAW)
		new(output_folder,POINTCLOUDS,FULL,PARTITIONS,DXF,RAW)
	end
end
