"""

"""
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

	Initializer(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64, k::Int64, outliers::Array{Int64,1}, current_inds::Array{Int64,1}) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), current_inds,Int64[])
	Initializer(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64, k::Int64, outliers::Array{Int64,1}) = new(PC, par, threshold, failed, N, k, outliers, copy(outliers), collect(1:PC.n_points),Int64[])
	Initializer(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64, k::Int64) = new(PC, par, threshold, failed, N, k, Int64[], Int64[], collect(1:PC.n_points),Int64[])
end


"""

"""
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
		FileManager.mkdir_if(POINTCLOUDS)

		FULL =  joinpath(POINTCLOUDS,"FULL")
		FileManager.mkdir_if(FULL)

		PARTITIONS = joinpath(POINTCLOUDS,"PARTITIONS")
		FileManager.mkdir_if(PARTITIONS)

		DXF = joinpath(output_folder,"DXF")
		FileManager.mkdir_if(DXF)

		RAW = joinpath(DXF,"RAW")
		FileManager.mkdir_if(RAW)

		new(output_folder,POINTCLOUDS,FULL,PARTITIONS,DXF,RAW)
	end
end



"""

"""
struct PlaneDirs
	output_folder::String
	PLANE::String
	A_SHAPES::String
	LINES::String

	function PlaneDirs(folder::String, project_name::String)
		output_folder = FileManager.mkdir_project(folder, project_name)

		PLANE = joinpath(output_folder,"PLANE")
		FileManager.mkdir_if(PLANE)

		A_SHAPES =  joinpath(output_folder,"A_SHAPES")
		FileManager.mkdir_if(A_SHAPES)

		LINES = joinpath(output_folder,"LINES")
		FileManager.mkdir_if(LINES)

		new(output_folder,PLANE,A_SHAPES,LINES)
	end
end
