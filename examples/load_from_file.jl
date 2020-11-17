#using Detection
using FileManager
using Common
using Visualization

fname = "C:/Users/marte/Documents/GEOWEB/TEST/PROFILE_Z250_XZ/PROFILE_Z250_XZ_vectorized_1D.txt"
source = "C:/Users/marte/Downloads/TEST_LINES_FITTING_ON_PROFILE/LA_CONTEA_PROFILE_DXF_Z_250_XZ/pointcloud_XZ_PLANE_SLICE_AT_QUOTE_16.7855_WITH_THICKNESS_0.05.las"


fname = "C:/Users/marte/Documents/GEOWEB/TEST/PROFILE_Z250_YZ/PROFILE_Z250_YZ_vectorized_1D.txt"
source = "C:/Users/marte/Downloads/TEST_LINES_FITTING_ON_PROFILE/LA_CONTEA_PROFILE_DXF_Z_250_YZ/pointcloud_YZ_PLANE_SLICE_AT_QUOTE_31.8827_WITH_THICKNESS_0.05.las"


fname = "C:/Users/marte/Documents/GEOWEB/TEST/PROFILE_Z650_YZ/PROFILE_Z650_YZ_vectorized_1D.txt"
source = "C:/Users/marte/Downloads/TEST_LINES_FITTING_ON_PROFILE/LA_CONTEA_PROFILE_DXF_Z_650_YZ/pointcloud_YZ_PLANE_SLICE_AT_QUOTE_33.4187_WITH_THICKNESS_0.05.las"

fname = "C:/Users/marte/Documents/GEOWEB/TEST/PROFILE_Z650_XZ/PROFILE_Z650_XZ_vectorized_1D.txt"
source = "C:/Users/marte/Downloads/TEST_LINES_FITTING_ON_PROFILE/LA_CONTEA_PROFILE_DXF_Z_650_XZ/pointcloud_XZ_PLANE_SLICE_AT_QUOTE_15.4525_WITH_THICKNESS_0.05.las"


V,EV = FileManager.load_segment(fname)
PC = FileManager.las2pointcloud(source)

GL.VIEW(
    [
    GL.GLGrid(V,EV,GL.COLORS[1],1.0),
    Visualization.points(PC.coordinates,GL.COLORS[2],0.2),

    GL.GLFrame2
    ]
)
