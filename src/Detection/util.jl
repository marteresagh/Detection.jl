
"""
remove from inds the i-th element described in todel
"""
function remove_points!(inds::Array{Int64,1},todel::Array{Int64,1})
	setdiff!(inds,todel)
end

"""
Assert the detected hyperplane is valid / interesting
"""
function validity(hyperplane::Hyperplane, params::Initializer)
	# VALIDITY
	pc_on_hyperplane = hyperplane.inliers
	@assert  pc_on_hyperplane.n_points > params.N "not valid"

	res = Common.residual(hyperplane).([c[:] for c in eachcol(pc_on_hyperplane.coordinates)])
	mu = Statistics.mean(res)# prova moda
	rho = Statistics.std(res)
	@assert mu+2*rho < params.par/2-0.005 || mu+2*rho > params.par/2+0.005 "not valid"  #0.005 che valore è?? come generalizzare??

end

"""
Corners detection
"""
function corners_detection(INPUT_PC::PointCloud, par::Float64, threshold::Float64, current_inds = collect(1:INPUT_PC.n_points)::Array{Int64,1})
	points = INPUT_PC.coordinates[:, current_inds]
	npoints = size(points,2)
	corners = fill(false,npoints)
	curvs = fill(0.,npoints)
	balltree = Common.BallTree(points)
	for i in 1:npoints
		# TODO verificare che i vicini ci siano e che il valore della curvatura non sia NaN
		N = Common.inrange(balltree, points[:,i], par, true) # usare un parametro abbastanza grande
		centroid = Common.centroid(points[:,N])
		C = zeros(2,2)
		for j in N
			diff = points[:,j] - centroid
			C += diff*diff'
		end

		eigval = Lar.eigvals(C)
		curvature = eigval[1]/sum(eigval)
		curvs[i] = curvature
	end
	#mu = Common.mean(curvs)
	for i in 1:npoints
		if  curvs[i] > threshold # TODO parametro da stimare in funzione dei dati.
			corners[i] = true
		end
	end

	return current_inds[corners], curvs
end
