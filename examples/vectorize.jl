using Detection
using OrthographicProjection
using Common


function get_planes()


    # XZ
    # steps = [0.,5,5,5,5,5,5,5,5,5,5]
    # p1 = [-0.3, 5.09, 9.0]
    # p2 = [52, 5.09, 9.0]

    #YZ
    steps = [0.0,5,5,5,5,5,5]
    p1 = [6.27, 62, 6.10]
    p2 = [6.27, -0.8, 6.10]
    axis_y = [0, 0, 1.]


    plan = Plane(p1,p2,axis_y)
    planes = Plane[]
    n_sections = length(steps)

    for i in 1:n_sections
        p_1 = Common.apply_matrix(Lar.t(-plan.matrix[1:3,3]*sum(steps[1:i])...),p1) # traslate model
        p_2 = Common.apply_matrix(Lar.t(-plan.matrix[1:3,3]*sum(steps[1:i])...),p2) # traslate model

        plan = Plane(vcat(p_1...),vcat(p_2...),axis_y)
        push!(planes,plan)
    end
    return planes
end

planes = get_planes()

for (root,dirs,files) in walkdir("C:/Users/marte/Documents/GEOWEB/TEST/LACONTEA_YZ")
    for file in files
        source = joinpath(root,file)
        output_folder =  "C:/Users/marte/Documents/GEOWEB/TEST/LACONTEA_YZ"
        n_section = parse(Int,split(splitext(file)[1],"_")[4])
        project_name = "Section_$n_section"
        plane = planes[n_section+1]
    	affine_matrix = plane.matrix
    	PC, threshold = Detection.source2pc(source, -1)
        threshold = 0.06
        N = 5
        k = 10
        par = 0.07
        failed = 10
    	Detection.pc2vectorize(output_folder, project_name, PC, par, threshold, failed, N, k, affine_matrix)
    end
end
