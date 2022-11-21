using Common
using Detection
using FileManager
using Visualization

source = raw"C:\Users\marte\Documents\GEOWEB\test\TEST\VECT2\VECT\DXF\RAW\segment2D.ext"
V1, EV1 = FileManager.load_segment(source)
proj_folder = raw"C:\Users\marte\Documents\GEOWEB\test\TEST\refine"
lines = FileManager.load_segment2lines(source)
Detection.refine_lines!(lines)

io = open(joinpath(proj_folder, "refineSegment2D.ext"), "w")
for i = 1:length(lines)
    line = lines[i]
    write(
        io,
        "$(line.startPoint[1]) $(line.startPoint[2]) $(line.endPoint[1]) $(line.endPoint[2])\n",
    )
end

close(io)


V2, EV2 = FileManager.load_segment(joinpath(proj_folder, "refineSegment2D.ext"))
Visualization.VIEW([
    Visualization.GLGrid(V2, EV2, Visualization.COLORS[3], 0.8),
    Visualization.GLGrid(V1,EV1, Visualization.COLORS[2], 0.5),
])
Visualization.VIEW([
    Visualization.GLGrid(V2, EV2, Visualization.COLORS[3], 0.8),
])

Visualization.VIEW([
    Visualization.GLGrid(V1,EV1, Visualization.COLORS[2], 0.8),
])



# 3 casi
lines = [
    Common.Line([1.0, 0.0], [5.0, 0.0]),
    Common.Line([5.1, 0.0], [8.0, 0.0]),
    Common.Line([10.0, 0.1], [13.0, 0.0]),
    Common.Line([12.0, 0.1], [15.0, 0.0]),
    Common.Line([16.0, 0.1], [19.0, 0.0]),
    Common.Line([16.0, 0.0], [19.0, 0.1]),
    Common.Line([1.0, 1.0], [1.0, 5.0]),
    Common.Line([1.0,5.1], [1.1, 8.0]),
    Common.Line([1.1, 5.1], [0.9, 8.0]),
    Common.Line([1.0, 9.0], [1.0, 13.0]),
    Common.Line([1.001, 12.0], [1.001, 16.0]),

]

lines = [Common.Line([1.1, 5.1], [0.9, 8.0]),
     # Common.Line([1.0, 0.0], [8.0, 0.0]),
     # Common.Line([14.999822222826925, -0.0077018551150376535], [10.000177777173075, 0.10770185511503766]),
     # Common.Line([16.0, 0.05], [19.0, 0.05]),
     Common.Line([0.9759221471195327, 1.0003130562451803], [1.0669365038172396, 8.00042988608739]),
     # Common.Line([1.000079999988096, 8.999999990400003], [1.0009200000119038, 16.000000009599997])
 ]
V1, EV1 = Common.DrawLines(lines)
Detection.refine_lines!(lines)
V2, EV2 = Common.DrawLines(lines)

Visualization.VIEW([
    Visualization.GLGrid(V2, EV2, Visualization.COLORS[3], 0.8),
    Visualization.GLGrid(V1,EV1, Visualization.COLORS[2], 0.5),
])

Visualization.VIEW([
    Visualization.GLGrid(V2, EV2, Visualization.COLORS[3], 0.8),
])

Visualization.VIEW([
    Visualization.GLGrid(V1,EV1, Visualization.COLORS[2], 0.5),
])


segments_lines = FileManager.load_segment2lines(source)
clusters = Detection.clustering_by_direction(segments_lines)
sort!(clusters; by = x->length(x), rev= true)
#salvo il cluster in un livello principale
#creo il livello secondario
max_element = length(clusters[1])
for cluster in clusters[2:end]
    if length(cluster)/max_element > 0.4
        #salvo nello stello livello del precedente
    else
        #salvo nel livello secondario
    end
end



V2, EV2 = Common.DrawLines(union(clusters[1],clusters[2]))

Visualization.VIEW([
    Visualization.GLGrid(V2, EV2, Visualization.COLORS[3], 0.8),
])
