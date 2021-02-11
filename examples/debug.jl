using Common
using FileManager
using Detection
using LightGraphs
using AlphaStructures
using Visualization

V = rand(2,1000)
# 2. applica alpha shape con alpha = threshold
DT = Common.delaunay_triangulation(V)
filtration = AlphaStructures.alphaFilter(V,DT);
threshold = 0.1
_, _, FV = AlphaStructures.alphaSimplex(V, filtration, threshold)

# 3. estrai bordo
EV_boundary = Common.get_boundary_edges(V,FV)
V,EV = Lar.simplifyCells(V,EV_boundary)

out = Lar.Struct([(V,EV),Lar.t(3,3),(V,EV)])
V,EV = Lar.struct2lar(out)

GL.VIEW([
	GL.GLGrid(V,EV,GL.COLORS[2],1.0),
])
