mutable struct Line
	direction::Array{Float64,1}
	centroid::Array{Float64,1}
end

struct LineDataset
    points::PointCloud
    line::Line
end

"""
Orthogonal distance.
"""
function distpointtoline(p::Array{Float64,1},line::Line)
	v = p - line.centroid
	p_star = v - Lar.dot(line.direction,v)*line.direction
	return Lar.norm(p_star)
end
"""
Check if point is close enough to model.
"""
function isclosetoline(p::Array{Float64,1},line::Line,par::Float64)
	return PointClouds.distpointtoline(p,line) < par
end

function LinesDetectionRandom(PC::PointCloud, par::Float64, threshold::Float64, failed::Int64, N::Int64)

	# 1. - initialization
	PCcurrent = deepcopy(PC)
	LINES = LineDataset[]
	linedetected = nothing

	f = 0
	i = 0

	# find lines
	PointClouds.flushprintln("======= Start search =======")
	search = true
	while search
		found = false

		while !found && f < failed
			try
				#PointClouds.flushprintln("ECCOMI")
				linedetected = PointClouds.LineDetectionFromRandomInitPoint(PCcurrent,par,threshold)

				# VALIDITY
				pointsonline = linedetected.points
				#@show linedetected
				@assert  pointsonline.n > N "not valid"  #da automatizzare
				# line = linedetected.line
				# E,_ = PointClouds.DrawLine(pointsonline.points, line, 0.0)
				# dist = Lar.norm(E[:,1]-E[:,2])
				# rho = pointsonline.n/dist
				# PointClouds.flushprintln("rho = $rho")
				# @assert  rho > N "not valid"  #da automatizzare

				found = true

			catch y

				f = f+1
				PointClouds.flushprintln("failed = $f")
				# if !isa(y, AssertionError)
				# 	notfound = false
				# end
			end
		end

		if found
			f = 0
			i = i+1
			PointClouds.flushprintln("$i lines found")
			push!(LINES,linedetected)
			deletePoints!(PCcurrent,linedetected.points)
		else
			search = false
		end

	end

	return LINES
end
