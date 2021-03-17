using Common
using FileManager
using Visualization
using Detection

folder_proj = "C:/Users/marte/Documents/GEOWEB/TEST"
NAME_PROJ = "MURI_LOD3"
folders = FileManager.get_plane_folders(folder_proj,NAME_PROJ)
boundary, full_boundary = FileManager.get_boundary(folders)
centroid = [291250.5043433152, 4.630341344699344e6, 106.74835850440863]

model = full_boundary[1] #11 12 16 17 20 28
V, EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)

GL.VIEW([
	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,model[1])[1:2,:])')),
#	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
	GL.GLGrid(Common.apply_matrix(Plane(V).matrix,V)[1:2,:],EV, GL.COLORS[2],1.)
])

# GL.VIEW([
# 	GL.GLPoints(convert(Lar.Points,P')),
# 	[GL.GLPoints(convert(Lar.Points,dict[i]'), GL.COLORS[rand(1:12)]) for i in keys(dict)]...,
# #	GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
# 	GL.GLGrid(P,EV, GL.COLORS[2],1.)
# ])
#

#
vect2D = Lar.LAR[]
for i in 1:length(full_boundary)
	@show i
	model = full_boundary[i]
	V,EV = Detection.simplify_model(model; par = 0.01, angle = pi/8)
	@show size(V)
	push!(vect2D,(V,EV))
	GL.VIEW([
		GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,model[1])[1:2,:])')),
		GL.GLPoints(convert(Lar.Points,(Common.apply_matrix(Plane(V).matrix,V)[1:2,:])'), GL.COLORS[2]),
		GL.GLGrid(Common.apply_matrix(Plane(V).matrix,V)[1:2,:],EV, GL.COLORS[2],1.)
	])
end

GL.VIEW(
	[GL.GLGrid(Common.apply_matrix(Lar.t(-centroid...),model[1]),model[2], GL.COLORS[rand(1:12)], 1.) for model in vect2D]
)
