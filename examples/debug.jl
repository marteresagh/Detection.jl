using Common
using FileManager
using Visualization
using Detection

W = FileManager.load_points("points.txt")
EW = FileManager.load_cells("edges.txt")
W = vcat(W,zeros(size(W,2))')
# GL.VIEW([
# 	#GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(W)...),W))),
# 	GL.GLGrid(W,EW,GL.COLORS[1],0.8),
# 	GL.GLFrame2
# ])

model = (W,EW)
V, EV, dict = Detection.simplify_model(model; par = 0.02, angle = pi/8)

plane = Plane(V)
V2D = Common.apply_matrix(plane.matrix,V)[1:2,:]
W2D = Common.apply_matrix(plane.matrix,W)[1:2,:]
# FileManager.save_connected_components("prova.txt",V,EV)
# EV = FileManager.load_connected_components("prova.txt")
GL.VIEW([
	GL.GLGrid(V2D,EV,GL.COLORS[1],0.8),
	GL.GLGrid(W2D,EW,GL.COLORS[2],0.8),
	[GL.GLPoints(permutedims(dict[k]), GL.COLORS[rand(1:12)]) for k in keys(dict)]...,
])

# GL.VIEW([
# 	[GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(V2D)...),dict[k])), GL.COLORS[rand(1:12)]) for k in keys(dict)]...,
# ])
# #
# p = dict[1]
# params = Common.LinearFit(p)
# hyperplane = Hyperplane(PointCloud(p),params...)
# Z,EZ = Common.DrawLines(hyperplane)
# GL.VIEW([
# 	GL.GLPoints(permutedims(Common.apply_matrix(Lar.t(-Common.centroid(p)...),W2D))),
# 	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(p)...),p)'),GL.RED),
# 	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(p)...),Z),EZ,GL.COLORS[1],1.0)
# ])
