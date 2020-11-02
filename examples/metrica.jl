using NearestNeighbors
using Visualization
using Common
using Detection


n = 1000
xs = 3*rand(n)
ys = Float64[]


for i in 1:n
    push!(ys, xs[i]*3 + 1+0.2*rand()) # points perturbation
end


for i in 1:n
    push!(ys, xs[i]*3 + 0.5 + 0.2*rand()) # points perturbation
end

V = convert(Lar.Points, hcat(vcat(xs,xs),ys)')

kdtree = KDTree(V)

i = rand(1:n)
rgb = rand(3,n)
idxs, dists = knn(kdtree,V[:,i],40)


GL.VIEW([
    GL.GLPoints(convert(Lar.Points,V'),GL.COLORS[1]),
    # GL.GLPoints(convert(Lar.Points,V[:,idxs]'),GL.COLORS[2]),
    # GL.GLPoints(convert(Lar.Points,V[:,[i,1]]'),GL.COLORS[12]),
])

par = 0.05
threshold = 2*0.1
failed = 10
N = 100
params = Initializer(PointCloud(V,rand(3,2*n)),par,threshold,failed,N)


hyperplanes = Detection.iterate_random_detection(params)

visual = Visualization.mesh_lines(hyperplanes)
GL.VIEW([visual...])

GL.VIEW([
    visual...,
    #GL.GLPoints(convert(Lar.Points,V'),GL.COLORS[1]),
    GL.GLPoints(convert(Lar.Points,V[:,params.current_inds]'),GL.COLORS[2]),
    # GL.GLPoints(convert(Lar.Points,V[:,[i,1]]'),GL.COLORS[12]),
])




# # ======================= INPUT generation === Semi-cerchio
# npoints = 2000
# angles = pi*rand(npoints)
#
# V = zeros(2,npoints)
#
# for i in 1:npoints
#     V[1,i] = cos(angles[i])+0.01*rand()
#     V[2,i] = sin(angles[i])+0.01*rand()
# end
#
# corr = Statistics.cor(V[2,:],V[1,:])
# PC = PointCloud(V,ones(3,npoints))
# GL.VIEW(
#     [
#     Visualization.points_color_from_rgb(PC.coordinates,PC.rgbs)
#     ]
# )
#
# par = 0.02
# threshold = 2*0.03
# failed = 400
# N = 100
# params = Initializer(PC,par,threshold,failed,N,10,[])
# hyperplanes, current_inds = Detection.iterate_random_detection(params)
#
# visual = Visualization.mesh_lines(hyperplanes)
# GL.VIEW([visual...])
#
#
#
# fname = "C:\\Users\\marte\\Documents\\GEOWEB\\FilePotree\\TEST_LINES\\prova\\PLANE\\PLANE_vectorized_1D.txt"
#
# V,EV = FileManager.load_segment(fname)
#
# GL.VIEW([
#
# 			GL.GLGrid(V,EV,GL.COLORS[1],1.0),
# 		])
#
