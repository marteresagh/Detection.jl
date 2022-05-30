using Detection
using Visualization
using Common
using FileManager
using Features
# using ProfileView
# using Statistics

function get_lines(hyperplanes)
	lines = Common.Line[]
	for line in hyperplanes
		V = Detection.extrema_line(line)
		push!(lines,Common.Line(V[:,1],V[:,2]))
	end
	return lines
end

# fname = "examples/las/wall.las"
fname = "examples/las/polyline.las"
# fname = "examples/las/full.las"
# fname = "examples/las/square.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/sezione_AMPHI_z39_5cm.las"
# fname = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/casaletto_planimetria.las"
# fname = "C:/Users/marte/Documents/GEOWEB/TEST/TEST NAVVIS/SEZIONE/SEZIONE_z=11_8.las"


PC = FileManager.las2pointcloud(fname)
PC_2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
INPUT_PC = Features.subsample(PC_2D, 0.01)
centroid = Common.centroid(INPUT_PC.coordinates)
Visualization.VIEW([
	Visualization.points(PC_2D.coordinates; color=Visualization.COLORS[2]),
	Visualization.points(INPUT_PC.coordinates; color=Visualization.COLORS[3])
])

# user - parameters
par = 0.05
failed = 100
N = 2
k = 30

# threshold estimation
threshold = 0.05  #Features.estimate_threshold(INPUT_PC,2*k)

# outliers
outliers = Features.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)

Visualization.VIEW([
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates[:,corners]); color = Visualization.BLUE),
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates[:,outliers]); color = Visualization.RED),
])

# process
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)

# masterseeds = "C:/Users/marte/Documents/GEOWEB/wrapper_file/JSON/seeds_sezione650.txt"
# given_seeds = FileManager.load_points(masterseeds)
# seeds = Common.consistent_seeds(INPUT_PC).([c[:] for c in eachcol(given_seeds)])
seeds = Int64[]

hyperplanes = Detection.iterate_detection(params; seeds = seeds, debug = true)
# hyperplane,_,_ = Detection.get_hyperplane(params)
# hyperplanes = [hyperplane]

lines = get_lines(hyperplanes)
V,EV = Common.DrawLines(lines)
Visualization.VIEW([
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates[:,params.visited]); color = Visualization.BLUE),
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates[:,params.outliers]); color = Visualization.RED),
	Visualization.points(Common.apply_matrix(Common.t(-centroid...),INPUT_PC.coordinates[:,params.fitted]); color = Visualization.GREEN),
	Visualization.GLGrid(Common.apply_matrix(Common.t(-centroid...),V),EV,Visualization.COLORS[1],1.0)
])

# GL.VIEW([
# 	#GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(INPUT_PC.coordinates)...),INPUT_PC.coordinates)'),GL.COLORS[2]),
# 	Visualization.mesh_lines(hyperplanes)...
# ])
