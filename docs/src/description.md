# Description
This package provides algorithms for shapes detection.
At present it is possible to detect hyperplanes: lines and planes (in 2D/3D space).

The main algorithm is dimension independent.
Let  $P$ be a point cloud, a cluster $R$ is a set of points of $P$ belonging at the same hyperplane.

Features of algorithm:
- Point test. Define a condition: for all $p$ in the set of unprocessed points, check if the distance from $p$ to fitting model is less of a fixed value $par$, then add $p$ to the cluster $R$.

- Seed points. It is useful remove points that can't be good seeds, for example outliers or points on sharp edges.

- Neighborhood. The algorithm is based on region growing method, we increase cluster by moving on the k-nearest neighbors of seeds.

- Termination. The process halts after a failed detection number in a row.

## Pseudocode
```
function iterate_random_detection(params::InitialParams)
    hyperplanes = Hyperplane[]
    f = 0
    search = true
    while search

        found = false
        while !found && f < params.failed
            try
                hyperplane, cluster, all_visited_verts = get_hyperplane_from_random_init_point(params)
                validity(hyperplane, params) # test of validity
                found = true
            catch y
                f = f+1
            end
        end

        if found
            f = 0
            push!(hyperplanes,hyperplane)
            union!(params.fitted,cluster)
            union!(params.visited,all_visited_verts)
        else
            search = false
        end

    end

    return hyperplanes
end
```
