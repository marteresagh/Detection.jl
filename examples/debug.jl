using Detection
using Visualization
using Common
using FileManager
using Statistics

#fname = "examples/wall.las"
#fname = "examples/muriAngolo.las"
#fname = "examples/area.las"
fname = "C:/Users/marte/Documents/GEOWEB/FilePotree/orthoCONTEA/Sezione_z650.las"
#fname = "C:/Users/marte/Documents/GEOWEB/FilePotree/AMPHI/sezione_AMPHI_z39_5cm.las"

PC = FileManager.las2pointcloud(fname)
PC2D = PointCloud(PC.coordinates[1:2,:], PC.rgbs)
# points, indx = Common.remove_double_verts(PC.coordinates[1:2,:],2)
# current_inds = collect(1:PC2D.n_points)
k = 20

outliers = Common.outliers(PC2D, collect(1:PC2D.n_points), k)
# da_tenere = setdiff(current_inds,outliers)

# GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,:]'),GL.COLORS[2]) ,
# 			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
#   			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,outliers]'),GL.COLORS[2]),
#
# 		])

par = 0.07
threshold = 2*0.03
failed = 100
N = 10
INPUT_PC = PointCloud(PC.coordinates[1:2,:],PC.rgbs)
params = Initializer(INPUT_PC,par,threshold,failed,N,k,outliers)
hyperplanes = Detection.iterate_random_detection(params)

# hyperplane,_,_ = Detection.get_hyperplane_from_random_init_point(params)

V,EV = Common.DrawLines(hyperplanes,0.0)
GL.VIEW([  	GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,:]'),GL.COLORS[2]) ,
			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
  			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
		])

GL.VIEW([ Visualization.mesh_lines(hyperplanes)...])
#
# GL.VIEW([  	GL.GLPoints(convert(Lar.Points,INPUT_PC.coordinates[:,params.fitted]'),GL.COLORS[2]) ,
# 			#GL.GLPoints(convert(Lar.Points,PC2D.coordinates[:,da_tenere]'),GL.COLORS[12]),
#   			GL.GLGrid(V,EV,GL.COLORS[1],1.0)
# 		])
#



function source2pc(source::String, plane::Detection.Plane, thickness::Float64)

	if isdir(source) # se source è un potree
		Detection.flushprintln("Potree struct")
		cloud_metadata = CloudMetadata(source)
		bbin = cloud_metadata.tightBoundingBox
		model = OrthographicProjection.Common.plane2model(plane,thickness,bbin)
		aabb = Detection.Common.boundingbox(model[1])
		mainHeader = Detection.FileManager.newHeader(aabb,"EXTRACTION",SIZE_DATARECORD)

		params = OrthographicProjection.ParametersExtraction("slice.las",
															[source],
															Matrix{Float64}(Detection.Lar.I,3,3),
															model,
															-Inf,
															Inf,
															mainHeader
															)

		OrthographicProjection.segment_and_save(params)

		return Detection.FileManager.las2pointcloud(params.outputfile)

	elseif isfile(source) # se source è un file
		Detection.flushprintln("Single file")
		bbin = Detection.FileManager.las2aabb(source)
		model = OrthographicProjection.Common.plane2model(plane,thickness,bbin)
		PC = Detection.FileManager.las2pointcloud(params.outputfile)

		if Common.modelsdetection(params.model, metadata.tightBoundingBox) == 2 # full model
			Detection.flushprintln("full model")
			return PC
		else
			Detection.flushprintln("slice")
			tokeep = Detection.Common.inmodel(model).([PC.coordinate[:,i] for i in 1:PC.n_points])
			return PointCloud(PC.coordinates[:,tokeep],PC.rgbs[:,tokeep])
		end

	end

end
source = "C:/Users/marte/Documents/GEOWEB/wrapper_file/sezioni/Sezione_z650.las"
plane = Detection.Plane(0,0,1,6.50)
thickness = 0.10
PC = source2pc(source,plane,thickness)
