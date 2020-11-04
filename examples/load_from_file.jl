using Detection
using FileManager
using Common
using Visualization

fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\prova\\SEZIONE_250LACONTEA\\SEZIONE_250LACONTEA_vectorized_1D.txt"
source = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\orthoCONTEA\\Sezione_z250.las"
V,EV = FileManager.load_segment(fname)
PC = FileManager.las2pointcloud(source)

GL.VIEW(
    [
    Visualization.points(PC.coordinates[1:2,:],GL.COLORS[2],1.0),
    GL.GLGrid(V[1:2,:],EV,GL.COLORS[1],1.0),
    GL.GLFrame2
    ]
)
