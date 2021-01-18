using Detection
using Visualization
using Common
using AlphaStructures
using FileManager
using LightGraphs

W = FileManager.load_points("point.txt")
EW = FileManager.load_cells("edges.txt")

L,EL = Detection.linearization(W,EW)

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),W)'),GL.COLORS[2]),
	GL.GLPoints(convert(Lar.Points,Common.apply_matrix(Lar.t(-Common.centroid(W)...),L)'),GL.COLORS[12]),
	GL.GLGrid(Common.apply_matrix(Lar.t(-Common.centroid(W)...),L),EL,GL.COLORS[rand(1:12)],1.0),
])
