using Common
using FileManager
using Detection

masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_CASALETTO.txt"
source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/CASALETTO"
INPUT_PC = FileManager.source2pc(source,2)

# user parameters
par = 0.06
failed = 100
N = 1000
k = 60

# threshold estimation
threshold = Common.estimate_threshold(INPUT_PC,k)

# normals
normals = Common.compute_normals(INPUT_PC.coordinates,threshold,k)
INPUT_PC.normals = normals

# seeds indices
given_seeds = FileManager.load_points(masterseeds)
seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

params = Initializer(INPUT_PC, par, threshold, failed, N, k)

# 2. Detection
hyperplanes = Detection.iterate_seeds_detection(params,seeds[2:end]; debug = true)
