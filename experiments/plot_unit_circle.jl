import Pkg
Pkg.activate(".")

using DistancePlan
using StaticArrays
import Makie

scene = Makie.Scene()

DistancePlan.draw!(Sphere(SVector{2}([0. 0.]), 1.))

Makie.save("unit_circle.png", scene, resolution=(1000, 1000))
