using Common
using Detection
using JSON
using Visualization
using Statistics
using AlphaStructures
using Features

project_folder = raw"C:\Users\marte\Documents\GEOWEB\PROGETTI\STANZA_ideale\provaprova"
potree = raw"C:\Users\marte\Documents\potreeDirectory\pointclouds\STANZA_IDEALE"



# read output CGAL
# final_points, final_faces = Detection.read_OFF(joinpath(output_folder, "output_faces.off"))
CGAL_folder = joinpath(project_folder, "SEGMENTS")
candidate_points, candidate_faces =
    Detection.read_OFF(joinpath(CGAL_folder, "candidate_faces.off"))
candidate_edges = Vector{Vector{Int}}[]
for face in candidate_faces
    edges = [[face[end],face[1]]]
    for i in 1:length(face)-1
        push!(edges,[face[i],face[i+1]])
    end
    push!(candidate_edges, edges )
end

model = (candidate_points, candidate_edges, candidate_faces)

function get_valid_faces(dict)
    tokeep = []
    for (k,v) in dict
        if haskey(v,"covered_area_percent") && v["covered_area_percent"] > 50
            push!(tokeep,k)
        end
    return tokeep
end

faces_folder = joinpath(project_folder, "tmp/FACES")
dict_faces = Dict{Int,Any}()

for dir in readdir(faces_folder)
    idx = parse(Int,split(dir,"_")[2])
    file = joinpath(faces_folder,dir,"faces.js")
	dict = nothing
    open(file, "r") do f
	    dict = JSON.parse(f)  # parse and transform data
	end

	dict_faces[idx] = dict
end
tokeep = get_valid_faces(dict_faces)
function getArea(V,FV)
	function triangle_area(triangle_points)
        ret = ones(3, 3)
        ret[:,1:2] = triangle_points'
        return Common.abs(0.5 * Common.det(ret))
    end

	area = 0.
	for face in FV
		ptri = V[:,face]
		area += triangle_area(ptri)
	end
	return area

end


k = 422
	if haskey(dict_faces[k], "vertices") && dict_faces[k]["n_points"] > 10
		model = hcat(dict_faces[k]["vertices"]...)
		plane = Common.Plane(model)
		filename = dict_faces[k]["points_on_face"]
		list_points = FileManager.load_points(filename)
			Visualization.VIEW([Visualization.points(list_points)])
		V2D_model = Common.apply_matrix(plane.matrix, model)
		V2D_points = Common.apply_matrix(plane.matrix, list_points)
		z = V2D_points[3,:]
		@show length(z)
		mu = 0. #Statistics.mean(z)
		mystd = Statistics.stdm(z,mu)
		tokeep = Int[]
		for i in 1:length(z)
			if z[i]>mu-mystd/2 && z[i]<mu+mystd/2
				push!(tokeep,i)
			end
		end
		points_plane = V2D_points[:,tokeep]
		Visualization.VIEW([Visualization.points(model)])
		Visualization.VIEW([Visualization.points(points_plane)])
		try
			filtration = AlphaStructures.alphaFilter(points_plane[1:2,:]);
			threshold = Features.estimate_threshold(points_plane[1:2,:],10)
			_,_,FV = AlphaStructures.alphaSimplex(points_plane[1:2,:], filtration,threshold)
			Visualization.VIEW([Visualization.GLGrid(points_plane[1:2,:],FV)])
			area_covered = getArea(points_plane[1:2,:],FV)
			covered_area_percent = (area_covered/dict_faces[k]["area"] )*100
			@show covered_area_percent
		catch
		end

	end
	# break


# clustering valid candidate faces
faces = candidate_faces[tokeep]
edges, triangles, regions =
    Detection.clustering_faces(candidate_points, faces)
mesh = []
for i in 1:21
	push!(mesh, Visualization.GLGrid(candidate_points,triangles[regions[i]],Visualization.COLORS[rand(1:12)]))
end
Visualization.VIEW(mesh)
# get polygons
polygons_folder = FileManager.mkdir_project(project_folder, "POLYGONS")
polygons = Detection.get_polygons(candidate_points, triangles, regions)

#save boundary polygons
if !isempty(polygons)
    Detection.save_boundary_polygons(
        polygons_folder,
        candidate_points,
        polygons,
    )
    FileManager.successful(
        true,
        project_folder;
        filename = "polygons_boundary.probe",
    )
end



for dir in readdir(faces_folder)
    idx = parse(Int,split(dir,"_")[2])
    file_model = joinpath(faces_folder,dir,"model.txt")
	include(file_model) #V,EV,FV
	file_points = joinpath(faces_folder,dir,"points_in_model.txt")
	if isfile(file_points)
		points_in_model = FileManager.load_points(file_points)
		Visualization.VIEW([
			Visualization.GLGrid(V,EV),
			Visualization.points(points_in_model)
		])
	end
end
