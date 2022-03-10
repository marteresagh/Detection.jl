using Common
using Visualization
using FileManager
source = raw"C:\Users\marte\Documents\GEOWEB\TEST\profili e sezioni\ortho.las"

PC = FileManager.source2pc(source)

M = [
    0.7962874020327942 9.158871777449771e-17 0.6049184848918597 6.749635026837064
    0.6049184848918614 -1.2056325596531764e-16 -0.7962874020327929 8.324092906993625
    -1.232595164407831e-32 1.0 -1.4348361284226481e-16 1.2477459907531778
    0 0 0 1
]


Visualization.VIEW([
    Visualization.points(Common.apply_matrix(M, PC.coordinates), PC.rgbs),
    Visualization.axis_helper()...,
])

segments = raw"C:\Users\marte\Documents\GEOWEB\TEST\profili e sezioni\vect1D\DXF\RAW\segment2D.ext"

V, EV = FileManager.load_segment(segments)

Visualization.VIEW([
    Visualization.GLGrid(V,EV),
    # Visualization.points(Common.apply_matrix(M, PC.coordinates), PC.rgbs),
    Visualization.axis_helper()...,
])
