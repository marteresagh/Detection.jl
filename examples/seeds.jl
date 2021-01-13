using Common
using FileManager
using Detection
using Visualization

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
hyperplanes = Detection.iterate_seeds_detection(params,seeds; debug = true)

centroid = Common.centroid(INPUT_PC.coordinates)
V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)

GL.VIEW([
			Visualization.points_color_from_rgb(Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates),INPUT_PC.rgbs),
			#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates)'),GL.COLORS[12]),
			#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-centroid...),INPUT_PC.coordinates[:,outliers])'),GL.COLORS[2]) ,
  			GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),V),FV,GL.COLORS[1],0.8)
		])
