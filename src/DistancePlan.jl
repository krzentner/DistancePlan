module DistancePlan

using StaticArrays

function norm(v)
  return sqrt(sum(v .^ 2))
end

function distance(center, obstacle::AbstractVector{T}) where {T <: AbstractFloat}
  return norm(center .- obstacle)
end

abstract type Volume end

struct Box <: Volume
  lows
  highs
end

export Box

function distance(center, obstacle::Box)
  diffs = [(obstacle.lows .- center) (center .- obstacle.highs)]
  return maximum(diffs)
end

struct Sphere <: Volume
  center
  radius
end

export Sphere

function distance(center, obstacle::Sphere)
  return norm(center .- obstacle.center) - obstacle.radius
end

function distance(center, obstacle::AbstractVector)
  return minimum([distance(center, obs) for obs=obstacle])
end

struct SphereTree
  centers::Vector
  squared_radii::Vector
  children::Vector{Vector{Int}}
  parent::Vector{Int}
  SphereTree() = new([], [], [], [])
end

export distance

function nearest_sphere(tree::SphereTree, point)::Int
  return argmin([sum((center .- point) .^ 2) - srad
                 for (center, srad) in zip(tree.centers, tree.squared_radii)])
end

function project(point, sphere::Sphere)
  v = point .- sphere.center
  v /= norm(v)
  v = v .* sphere.radius .+ sphere.center
  draw!(v, color="orange")
  return v
end

function sample(bounds::Box)::SVector{N} where {N}
  r = rand(N)
  x = SVector{N}((bounds.highs .- bounds.lows) .* r .+ bounds.lows)
  draw!(x, color="blue")
  return x
end

function push_sphere!(tree::SphereTree, parent::Int, sphere::Sphere)
  child = 1 + length(tree.parent)
  push!(tree.centers, sphere.center)
  push!(tree.squared_radii, sphere.radius ^ 2)
  push!(tree.parent, parent)
  push!(tree.children, [])
  push!(tree.children[parent], child)
end

function get_sphere(tree::SphereTree, sphere::Int)
  return Sphere(tree.centers[sphere], sqrt(tree.squared_radii[sphere]))
end

function evt_step!(tree::SphereTree, bounds::Box, obstacles::Vector; min_radius=10.) where {N}
  x = sample{N}(bounds)
  x_near = nearest_sphere(tree, x)
  x_new = project(x, get_sphere(tree, x_near))
  x_new_rad = distance(x_new, obstacles)
  # Sphere is of minimum radius and center is in bounds.
  # Center can be out of bounds due to projection from inside sphere.
  if x_new_rad > min_radius && distance(x_new, bounds) < 0
    push_sphere!(tree, x_near, Sphere(x_new, x_new_rad))
  end
end

function evt(origin::SVector, bounds::Box, obstacles::Vector, steps::Int) where {N}
  tree = SphereTree()
  radius = distance(origin, obstacles)
  origin_sphere = Sphere(origin, radius)
  push_sphere!(tree, 1, origin_sphere)
  for iter in 1:steps
    evt_step!{N}(tree, bounds, obstacles)
  end
  return tree
end

export evt

function get_path(tree::SphereTree, last_idx::Int)::Vector
  path = [tree.centers[last_idx]]
  while last_idx != 1
    last_idx = tree.parent[last_idx]
    pushfirst!(path, tree.centers[last_idx])
  end
  return path
end

function evt_goal(origin::SVector, bounds::Box, obstacles::Vector, goal::SVector; min_radius=10.)
  tree = SphereTree()
  radius = distance(origin, obstacles)
  origin_sphere = Sphere(origin, radius)
  push_sphere!(tree, 1, origin_sphere)
  while true
    gidx = nearest_sphere(tree, goal)
    goal_sphere = Sphere(tree.centers[gidx], sqrt(tree.squared_radii[gidx]))
    if distance(goal, goal_sphere) <= 0
      return tree, get_path(tree, gidx)
    end
    evt_step!(tree, bounds, obstacles, min_radius=min_radius)
  end
end

export evt_goal

import Makie

function draw!(box::Box; color="red")
  lx, ly = box.lows
  hx, hy = box.highs
  points = [
      lx ly
      hx ly
      hx hy
      lx hy
      lx ly
      NaN NaN
  ]
  Makie.plot!(points[:, 1], points[:, 2], color=color)
end

function draw!(sphere::Sphere; points=64, color="red")
  cx, cy = sphere.center
  r = sphere.radius
  t = range(0, 2pi, length=points)
  x = cx .+ r .* cos.(t)
  y = cy .+ r .* sin.(t)
  Makie.plot!(x, y, color=color)
end

function draw!(obstacle::SVector; delta=0.1, color="red")
  x, y = obstacle
  points = [
      x - delta y - delta
      x + delta y + delta
      NaN NaN
      x - delta y + delta
      x + delta y - delta
      NaN NaN
  ]
  Makie.plot!(points[:, 1], points[:, 2], color=color)
end

function draw!(tree::SphereTree; color="green")
  for (center, srad) in zip(tree.centers, tree.squared_radii)
    draw!(Sphere(center, sqrt(srad)), color=color)
  end
end

export draw!

end # module
