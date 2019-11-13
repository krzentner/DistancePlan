import Pkg
Pkg.activate(".")

using DistancePlan
using StaticArrays
import Makie

scene = Makie.Scene()

B = 3.
O = 200
S = 200
origin = SVector{2}(0.5, 0.5)

bounds = DistancePlan.Box(SVector{2}([-B -B]), SVector{2}([B B]))
R = 2 * B
sample_bounds = DistancePlan.Box(SVector{2}([-R -R]), SVector{2}([R R]))
obstacles = [DistancePlan.sample(bounds) for _ in 1:O]

DistancePlan.evt(origin, bounds, obstacles, 2)

println("Gathering ", S, " samples")
tree = DistancePlan.evt(origin, bounds, obstacles, S)
println("Built tree")

DistancePlan.draw!(bounds)
DistancePlan.draw!.(obstacles)
DistancePlan.draw!(tree)

Makie.save("random_tree.png", scene, resolution=(1000, 1000))
