using Detection
using Test

@testset "lines_detection" begin
	par = 0.07
	failed = 100
	N = 10
	k = 30

	workdir = dirname(@__FILE__)
	fname = joinpath(workdir,"model/polyline.las")
	PC = Detection.FileManager.las2pointcloud(fname)
	INPUT_PC = Detection.PointCloud(PC.coordinates[1:2,:], PC.rgbs)
	threshold = Detection.Features.estimate_threshold(INPUT_PC,2*k)
	outliers = Detection.Features.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	params = Detection.Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
	seeds = Int64[]
	hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)
	@test length(hyperplanes)>3
end


@testset "planes_detection" begin
	par = 0.04
	failed = 100
	N = 10
	k = 30

	workdir = dirname(@__FILE__)
	fname = joinpath(workdir,"model/MURI.las")
	INPUT_PC = Detection.FileManager.las2pointcloud(fname)
	threshold = Detection.Features.estimate_threshold(INPUT_PC,2*k)
	normals = Features.compute_normals(INPUT_PC.coordinates, threshold, k)
	INPUT_PC.normals = normals
	outliers = Detection.Features.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
	params = Detection.Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
	seeds = Int64[]
	hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)
	@test length(hyperplanes)>0
end


@testset "simplify_model" begin
	workdir = dirname(@__FILE__)
	fname = joinpath(workdir,"model/model.txt")
	include(fname)
	model = V,EV
	W, EW = Detection.simplify_model(model; par = 0.02, angle = pi/8)
	@test size(W,2)==4
	@test length(EW)==4
end
