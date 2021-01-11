# Detection

`Detection.jl` is a [Julia](http://julialang.org) library for shape detection.

# Getting started

## Installation

To install a Julia package you have to use the package manager Pkg. Enter the Pkg REPL by pressing ] from the Julia REPL and then use the command `add`.

This package is not in a registry, it can be added by instead of the package name giving the URL to the repository to add.

 ```julia
 (@v1.4) pkg> add https://github.com/marteresagh/Detection.jl
 ```

## Dependencies

`OrthographicProjection.jl` has the following dependencies:
 - [```Common.jl```](https://github.com/marteresagh/Common.jl)
 - [```FileManager.jl```](https://github.com/marteresagh/FileManager.jl)

# Structure of documentation

This documentation is a collection of several parts:

* The "Description" part explains the algorithm implemented.
* The "How to" part gives you demonstrations of how to carry out specific tasks with Detection.
* The "Script" part describes how to launch a job from command line.
* The "Results" part shows you the results of a specific job.
* The "References" part is a collection of API function references provided by Detection.
