# Description
This package provides algorithms for shapes detection.
At present it is possible to detect hyperplanes: lines and planes (in 2D/3D space).

The main algorithm is dimension independent.
Let  $P$ be a point cloud, a cluster $R$ is a set of points of $P$ belonging at the same hyperplane.

Features of algorithm:
- Point test. Let's define a condition: for all $p$ in the set of unprocessed points, $p$ is added to the cluster $R$ if the distance from $p$ to fitting model is less of a fixed value $par$.

- Seed points. The seeds are chosen randomly and it is useful to remove points that may not be good seeds, such as outliers or points on sharp edges. Seeds can be provided by the user too.

- Neighborhood. The algorithm is based on region growing method, we increase clusters by adding the k-nearest neighbors of seeds that meet the condition of point test.

- Validity. A criterion to validate found hyperplanes.

- Termination. The process halts after a failed detection number in a row.

## Pseudocode
```
function iterate_random_detection(params::Initializer)
    hyperplanes = Hyperplane[]
    f = 0
    search = true
    while search

        found = false
        while !found && f < params.failed
            try
                hyperplane, cluster, all_visited_verts = get_hyperplane(params)
                validity(hyperplane, params) # test of validity
                found = true
            catch y
                f = f+1
            end
        end

        if found
            f = 0
            push!(hyperplanes,hyperplane)
        else
            search = false
        end

    end

    return hyperplanes
end


function iterate_seeds_detection(params::Initializer, seeds::Array{Int64,1})
	hyperplanes = Hyperplane[]

	for seed in seeds
		found = false

		try
			hyperplane, cluster, all_visited_verts = get_hyperplane(params; given_seed = seed)
			found = true
		catch y
			# not valid seed
		end

		if found
			push!(hyperplanes,hyperplane)
		end

	end

	return hyperplanes
end
```
