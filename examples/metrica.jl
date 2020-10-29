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
