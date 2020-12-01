using Detection
using Visualization
using Common
using FileManager
using Statistics

source = "C:/Users/marte/Documents/potreeDirectory/pointclouds/PAVIMENTO"
INPUT_PC,threshold = Detection.source2pc(source,2)
par = 0.2
#threshold = 2*0.03
failed = 10
N = 100
k = 30
outliers = Common.outliers(INPUT_PC, collect(1:INPUT_PC.n_points), k)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params;debug = true)
#hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)
V,FV = Common.DrawPlanes(hyperplanes, nothing, 0.0)
GL.VIEW([
			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[12]),
			# GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]) ,
  			GL.GLGrid(V,FV,GL.COLORS[1],1.0)
		])


GL.VIEW([	#GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,:]'),GL.COLORS[2]),
			Visualization.mesh_lines(hyperplanes)...,GL.GLGrid(V,FV,GL.COLORS[1],1.0)])


GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates'),GL.COLORS[1]) ,
  			GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,outliers]'),GL.COLORS[2]),
		])
