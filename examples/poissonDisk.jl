using Common
using Detection
using Visualization
using Distributions
using FileManager


function poisson_disk_rand_points(k,r,width,height)
    function get_cell_coords(pt, a)
        return Int(floor(pt[1] / a)), Int(floor(pt[2] / a))
    end

    function get_neighbours(coords, cells)
        dxdy = [(-1,-2),(0,-2),(1,-2),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),
                (-2,0),(-1,0),(1,0),(2,0),(-2,1),(-1,1),(0,1),(1,1),(2,1),
                (-1,2),(0,2),(1,2),(0,0)]
        neighbours = []
        for (dx, dy) in dxdy
            neighbour_coords = coords[1] + dx, coords[2] + dy
            if !(0 <= neighbour_coords[1] < nx && 0 <= neighbour_coords[2] < ny)
                # We're off the grid: no neighbours here.
                continue
            end
            neighbour_cell = cells[neighbour_coords]
            if !isnothing(neighbour_cell)
                # This cell is occupied: store this index of the contained point.
                append!(neighbours,neighbour_cell)
            end
        end
        return neighbours
    end


    function point_valid(pt, cells, a, samples)
        cell_coords = get_cell_coords(pt, a)
        for idx in get_neighbours(cell_coords,cells)
            nearby_pt = samples[idx]
            # Squared distance between or candidate point, pt, and this nearby_pt.
            distance2 = (nearby_pt[1] - pt[1])^2 + (nearby_pt[2] - pt[2])^2
            if distance2 < r^2
                # The points are too close, so pt is not a candidate.
                return false
                # All points tested: if we're here, pt is valid
            end
        end
        return true
    end

    function get_point(k, refpt)
        i = 0
        while i < k
            rho, theta = rand(Uniform(r, 2*r)), rand(Uniform(0, 2*pi))
            pt = refpt[1] + rho*Common.cos(theta), refpt[2] + rho*Common.sin(theta)
            if !(0 <= pt[1] < width && 0 <= pt[2] < height)
                    # This point falls outside the domain, so try again.
                continue
            end
            if point_valid(pt, cells, a, samples)
                return pt
            end
            i += 1
            # We failed to find a suitable point in the vicinity of refpt.
        end
        return nothing
    end

    a = r/Common.sqrt(2)
    # Number of cells in the x- and y-directions of the grid
    nx, ny = Int(floor(width / a)) + 1, Int(floor(height / a)) + 1

        # A list of coordinates in the grid of cells
    coords_list = [(ix, iy) for ix in range(1,stop=nx) for iy in range(1,stop=ny)]
        # Initilalize the dictionary of cells: each key is a cell's coordinates, the
        # corresponding value is the index of that cell's point's coordinates in the
        # samples list (or None if the cell is empty).
    cells = Dict()
    for coords in coords_list
        cells[coords] = nothing
    end
        # Pick a random point to start with.
    pt = rand(Uniform(0, width)),rand(Uniform(0, height))
    samples = [pt]
        # Our first sample is indexed at 0 in the samples list...
    cells[get_cell_coords(pt, a)] = 1
        # ... and it is active, in the sense that we're going to look for more points
        # in its neighbourhood.
    active = [1]

    nsamples = 1
        # As long as there are points in the active list, keep trying to find samples.
    while !isempty(active)
            # choose a random "reference" point from the active list.
        idx = rand(1:length(active))
        refpt = samples[idx]
            # Try to pick a new point relative to the reference point.
        pt = get_point(k, refpt)
        if !isnothing(pt)
                # Point pt is valid: add it to the samples list and mark it as active
            push!(samples,pt)
            nsamples += 1
            append!(active, length(samples))
            cells[get_cell_coords(pt,  a)] = length(samples)
        else
                # We had to give up looking for valid points near refpt, so remove it
                # from the list of "active" points.
            deleteat!(active, idx)
        end
    end

    return samples
end
k = 100

# Minimum distance between samples
r = 0.002

width, height = 20, 20

sample = poisson_disk_rand_points(k,r,width,height)

points = hcat(collect.(sample)...)

Visualization.VIEW([Visualization.points(points)])

points2Ds = [rand(2,20) for i in 1:1000]
function test(points2Ds)
    Threads.@threads for i in 1:1000
        @show i
        points2D = points2Ds[i]
        filtration = AlphaStructures.alphaFilter(points2D);
        threshold = Features.estimate_threshold(points2D,10)
        _,_,FV = AlphaStructures.alphaSimplex(points2D, filtration,threshold)
        @show i, length(FV)
    end
end

@time test(points2Ds)




using PyCall

function PoissonDisc(width=50, height=50, r=1, k=30)
    py"""
    import numpy as np
    import matplotlib.pyplot as plt

    # For mathematical details of this algorithm, please see the blog
    # article at https://scipython.com/blog/poisson-disc-sampling-in-python/
    # Christian Hill, March 2017.

    class PoissonDisc():

        def __init__(self, width=50, height=50, r=1, k=30):
            self.width, self.height = width, height
            self.r = r
            self.k = k

            # Cell side length
            self.a = r/np.sqrt(2)
            # Number of cells in the x- and y-directions of the grid
            self.nx, self.ny = int(width / self.a) + 1, int(height / self.a) + 1

            self.reset()

        def reset(self):

            # A list of coordinates in the grid of cells
            coords_list = [(ix, iy) for ix in range(self.nx)
                                    for iy in range(self.ny)]
            # Initilalize the dictionary of cells: each key is a cell's coordinates
            # the corresponding value is the index of that cell's point's
            # coordinates in the samples list (or None if the cell is empty).
            self.cells = {coords: None for coords in coords_list}

        def get_cell_coords(self, pt):


            return int(pt[0] // self.a), int(pt[1] // self.a)

        def get_neighbours(self, coords):

            dxdy = [(-1,-2),(0,-2),(1,-2),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),
                    (-2,0),(-1,0),(1,0),(2,0),(-2,1),(-1,1),(0,1),(1,1),(2,1),
                    (-1,2),(0,2),(1,2),(0,0)]
            neighbours = []
            for dx, dy in dxdy:
                neighbour_coords = coords[0] + dx, coords[1] + dy
                if not (0 <= neighbour_coords[0] < self.nx and
                        0 <= neighbour_coords[1] < self.ny):
                    # We're off the grid: no neighbours here.
                    continue
                neighbour_cell = self.cells[neighbour_coords]
                if neighbour_cell is not None:
                    # This cell is occupied: store the index of the contained point
                    neighbours.append(neighbour_cell)
            return neighbours

        def point_valid(self, pt):

            cell_coords = self.get_cell_coords(pt)
            for idx in self.get_neighbours(cell_coords):
                nearby_pt = self.samples[idx]
                # Squared distance between candidate point, pt, and this nearby_pt.
                distance2 = (nearby_pt[0]-pt[0])**2 + (nearby_pt[1]-pt[1])**2
                if distance2 < self.r**2:
                    # The points are too close, so pt is not a candidate.
                    return False
            # All points tested: if we're here, pt is valid
            return True

        def get_point(self, refpt):

            i = 0
            while i < self.k:
                rho, theta = (np.random.uniform(self.r, 2*self.r),
                              np.random.uniform(0, 2*np.pi))
                pt = refpt[0] + rho*np.cos(theta), refpt[1] + rho*np.sin(theta)
                if not (0 <= pt[0] < self.width and 0 <= pt[1] < self.height):
                    # This point falls outside the domain, so try again.
                    continue
                if self.point_valid(pt):
                    return pt
                i += 1
            # We failed to find a suitable point in the vicinity of refpt.
            return False

        def sample(self):

            # Pick a random point to start with.
            pt = (np.random.uniform(0, self.width),
                  np.random.uniform(0, self.height))
            self.samples = [pt]
            # Our first sample is indexed at 0 in the samples list...
            self.cells[self.get_cell_coords(pt)] = 0
            # and it is active, in the sense that we're going to look for more
            # points in its neighbourhood.
            active = [0]

            # As long as there are points in the active list, keep looking for
            # samples.
            while active:
                # choose a random "reference" point from the active list.
                idx = np.random.choice(active)
                refpt = self.samples[idx]
                # Try to pick a new point relative to the reference point.
                pt = self.get_point(refpt)
                if pt:
                    # Point pt is valid: add it to samples list and mark as active
                    self.samples.append(pt)
                    nsamples = len(self.samples) - 1
                    active.append(nsamples)
                    self.cells[self.get_cell_coords(pt)] = nsamples
                else:
                    # We had to give up looking for valid points near refpt, so
                    # remove it from the list of "active" points.
                    active.remove(idx)

            return self.samples

    """
    PD = py"PoissonDisc"(width,height,r,k)
    return hcat(collect.(PD.sample())...)
end

points = PoissonDisc(20, 20, 0.02, 30)

Visualization.VIEW([Visualization.points(points)])
