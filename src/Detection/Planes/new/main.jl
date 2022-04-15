"""
	pc2plane(
		folder::String,
		project_name::String,
		PC::PointCloud,
		par::Float64,
		failed::Int64,
		N::Int64,
		k::Int64;
		masterseeds = nothing::Union{String,Nothing}
		)

Main program.
Detect hyperplanes in point cloud.
"""
function vect2D(
    planes_folder::String,
    pc_folder::String,
    PC::PointCloud,
    par::Float64,
    failed::Int64,
    N::Int64,
    k::Int64;
    masterseeds = nothing::Union{String,Nothing},
)

    # seeds
    seeds = Int64[]
    if !isnothing(masterseeds) # if seeds are provided
        println("Read seeds from file")
        given_seeds = FileManager.load_points(masterseeds)
        seeds =
            Search.consistent_seeds(PC).([c[:] for c in eachcol(given_seeds)])
    end

    params = Detection.Initializer(PC, par, failed, N, k)

    # 2. Detection
    println()
    println("=========== PROCESSING =============")
    i = Detection.iterate_planes_detection(params, planes_folder; seeds = seeds)

    # 3. Saves
    println()
    println("=========== RESULTS =============")
    println("$i lines detected")

    if i != 0
        # Saving points
        print("Saving: Fitted and unfitted points... ")
        point_cloud = params.PC

        fitted_idx = params.fitted
        if !isempty(fitted_idx)
            PC_fitted = Detection.PointCloud(
                point_cloud.coordinates[:, fitted_idx],
                point_cloud.rgbs[:, fitted_idx],
            )
            FileManager.save_points_rgbs_txt(
                joinpath(pc_folder, "fitted_points.txt"),
                PC_fitted,
            )
            FileManager.save_pointcloud(
                joinpath(pc_folder, "fitted_points.las"),
                PC_fitted,
                "PLANES DETECTION",
            )
        end

        unfitted_idx = setdiff(collect(1:point_cloud.n_points), fitted_idx)
        if !isempty(unfitted_idx)
            PC_unfitted = Detection.PointCloud(
                point_cloud.coordinates[:, unfitted_idx],
                point_cloud.rgbs[:, unfitted_idx],
            )
            FileManager.save_points_rgbs_txt(
                joinpath(pc_folder, "unfitted_points.txt"),
                PC_unfitted,
            )
            FileManager.save_pointcloud(
                joinpath(pc_folder, "unfitted_points.las"),
                PC_unfitted,
                "PLANES DETECTION",
            )
        end
    end

    params.PC = Common.PointCloud()
    params.hyperplanes = i
    return params
end
