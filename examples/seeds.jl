using Common
using FileManager
using Detection

masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_CASALETTO.txt"

threshold = Common.estimate_threshold(INPUT_PC,k)

# seeds indices
given_seeds = FileManager.load_points(masterseeds)
seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])

params = Initializer(INPUT_PC, par, threshold, failed, N, k, outliers)

# 2. Detection
hyperplanes = Detection.iterate_seeds_detection(params,seeds)
