# Script Usage

# VECT 1D

## vectorize_1D.jl

Lines detection.

Detect lines in a 3D flat point cloud, i.e. points that lies on an arbitrary plane `π` (such as floor plan), in the 3D space.
An useful parameters is the description of plane `π` since during the process it is necessary to convert 3D points to 2D coordinates.

#### Input parameters description:
 - source: Potree project of slice
 - lod: level of detail of Potree
 - output: output folder
 - projectname: name of project
 - par: minimum distance to belong to line
 - failed: number of failed in a row
 - validity: minimum number of inliers
 - k: number of neighbors
 - plane: Hessian form of plane describing slice (`ax+by+cz=d`)
 - masterseeds: a text file with input seeds list

The output files are organized like this:
```
name_of_project
|--POINTCLOUDS
    |--FULL
        |--slice.las
    |--PARTITIONS
        |--fitted.las
        |--unfitted.las
|--DXF
    |--RAW
        |--segment3D.ext
        |--fitted3D.pnt
        |--unfitted3D.pnt
        |--segment2D.ext
        |--fitted2D.pnt
        |--unfitted2D.pnt
```

#### Options:
```
$ julia vectorize_1D.jl -h

usage: vectorize_1D.jl -p PROJECTNAME -o OUTPUT --par PAR [--lod LOD]
                       [--failed FAILED] [--validity VALIDITY] [--k K]
                       --plane PLANE [-s MASTERSEEDS] [-h] source

positional arguments:
  source                Potree directory

optional arguments:
  -p, --projectname PROJECTNAME
                        Project name
  -o, --output OUTPUT   Output folder
  --par PAR             Distance to line (type: Float64)
  --lod LOD             Level of detail. If -1, all points are taken
                        (type: Int64, default: -1)
  --failed FAILED       Number of failed before exit (type: Int64,
                        default: 100)
  --validity VALIDITY   Minimum number of inliers (type: Int64,
                        default: 5)
  --k K                 Number of neighbors (type: Int64, default: 30)
  --plane PLANE         a, b, c, d parameters described the plane
  -s, --masterseeds MASTERSEEDS
                        A text file with seeds list
  -h, --help            show this help message and exit
```

#### Examples:

    # lines detection
    julia vectorize_1D.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "FLOOR_PLAN" --par 0.02 --plane "0 0 1 0"

    # lines detection with initial seeds provided by users
    julia vectorize_1D.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "FLOOR_PLAN" --par 0.02 --plane "0 0 1 0" --masterseeds "C:/seeds.txt"


# VECT 2D

Follow this pipeline:
 1. `vectorize_2D.jl`: planes detection,
 2. `vectorize_2D_segment.jl`: segment full point cloud,
 3. `vectorize_2D_boundary.jl`: draw flat shapes.

The output files are organized in folders, one for each planes detected:
```
name_of_project
|--plane_$timestamp
    |--finite_plane.txt
    |--inliers.txt
    |--full_inliers.las
    |--boundary_points2D.txt
    |--boundary_points3D.txt
    |--boundary_edges.txt
|--plane_$timestamp
    ...
```

## 1. vectorize_2D.jl

Planes detection: return description of planes detected in a 3D point cloud.

#### Input parameters description:
 - source: Potree project
 - lod: level of detail of Potree
 - output: output folder
 - projectname: name of project
 - par: minimum distance to plane
 - failed: number of failed in a row
 - validity: minimum number of inliers
 - k: number of neighbors
 - masterseeds: a text file with input seeds list

#### Output:
 - `finite_plane.txt`: a,b,c,d (`ax+by+cz=d`) and scale, center and rotation of OBB,
 - `inliers.txt`: coordinates and rgbs.

#### Options:
```
$ julia vectorize_2D.jl -h

usage: vectorize_2D.jl -p PROJECTNAME -o OUTPUT --par PAR [--lod LOD]
                       [--failed FAILED] [--validity VALIDITY] [--k K]
                       [-s MASTERSEEDS] [-h] source

positional arguments:
  source                Input Potree

optional arguments:
  -p, --projectname PROJECTNAME
                        Project name
  -o, --output OUTPUT   Output folder
  --par PAR             Parameter (type: Float64)
  --lod LOD             Level of detail. If -1, all points are taken
                        (type: Int64, default: 1)
  --failed FAILED       number of failed before exit (type: Int64,
                        default: 100)
  --validity VALIDITY   number of points in a line (type: Int64,
                        default: 40)
  --k K                 number of neighbors (type: Int64, default: 30)
  -s, --masterseeds MASTERSEEDS
                        A text file with seeds list
  -h, --help            show this help message and exit
```

#### Examples:

    # planes detection
    julia vectorize_2D.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "PLANES" --par 0.02

    # planes detection with initial seeds provided by users
    julia vectorize_2D.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "PLANES" --par 0.02 --masterseeds "C:/seeds.txt"


## 2. vectorize_2D_segment.jl

Segment full point cloud: return all points lying on planes detected at first step.

#### Input parameters description:
 - source: Potree project
 - output: output folder (as first step)
 - projectname: name of project (as first step)
 - thickness: thickness of slice

#### Output:
 - `full_inliers.las`: LAS file.

#### Options:
```
$ julia vectorize_2D_segment.jl -h

usage: vectorize_2D_segment.jl -p PROJECTNAME -o OUTPUT
                        [--thickness THICKNESS] [-h] source

positional arguments:
  source                Input Potree

optional arguments:
  -p, --projectname PROJECTNAME
                        Project name
  -o, --output OUTPUT   Output folder
  --thickness THICKNESS
                        Thickness (type: Float64)
  -h, --help            show this help message and exit
```

#### Examples:

    # segmentation
    julia vectorize_2D_segment.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "PLANES" --thickness 0.02


## 2. vectorize_2D_boundary.jl

Return a simplify model of boundary edges of flat shapes.

#### Input parameters description:
 - source: Potree project
 - output: output folder (as first step)
 - projectname: name of project (as first step)
 - par: minimum distance from edge (clustering)
 - angle: minimum angle for linearity (clustering)
 - k: number of neighbors (threshold alpha shapes)

#### Output:
 - `boundary_points2D.txt`: 2D points of model,
 - `boundary_points3D.txt`: 3D points of model,
 - `boundary_edges.txt`: sequence of points, one component per row.

#### Options:
```
$ julia vectorize_2D_boundary.jl -h

usage: vectorize_2D_boundary.jl -p PROJECTNAME -o OUTPUT [--par PAR]
                        [--angle ANGLE] [--k K] [-h] source

positional arguments:
  source                Input Potree

optional arguments:
  -p, --projectname PROJECTNAME
                        Project name
  -o, --output OUTPUT   Output folder
  --par PAR             Parameter (type: Float64, default: 0.02)
  --angle ANGLE         Angle (type: Float64, default: 0.392699)
  --k K                 number of neighbors (type: Int64, default: 40)
  -h, --help            show this help message and exit
```

#### Examples:

    # segmentation
    julia vectorize_2D_segment.jl "C:/POTREE_PROJECT" -o "C:/MY_PROJS" -p "PLANES"
