module DistancePlan

using StaticArrays

function norm(v::SVector{N, T})::T where {N, T <: AbstractFloat}
  return sqrt(sum(v .^ 2))
end

function distance(center::SVector{N, T}, obstacle::SVector{N, T})::T where {N, T <: AbstractFloat}
  return norm(center .- obstacle)
end

abstract type Volume end

struct Box{N, T <: AbstractFloat} <: Volume
  lows::SVector{N, T}
  highs::SVector{N, T}
end

export Box

function distance(center::SVector{N, T}, obstacle::Box{N, T})::T where {N, T <: AbstractFloat}
  diffs = [(obstacle.lows .- center) (center .- obstacle.highs)]
  return maximum(diffs)
  # max_diffs = maximum(diffs, dims=2)
  # return maximum(max_diffs)
  # return minimum(max_diffs) * sign(maximum(max_diffs))
end

struct Sphere{N, T <: AbstractFloat} <: Volume
  center::SVector{N, T}
  radius::T
end

export Sphere

function distance(center::SVector{N, T}, obstacle::Sphere{N, T})::T where {N, T <: AbstractFloat}
  return norm(center .- obstacle.center) - obstacle.radius
end

function distance(center::SVector{N, T}, obstacle::AbstractVector)::T where {N, T <: AbstractFloat}
  return minimum([distance(center, obs) for obs=obstacle])
end

struct SphereTree{N, T <: AbstractFloat}
  centers::Vector{SVector{N, T}}
  squared_radii::Vector{T}
  children::Vector{Vector{Int}}
  parent::Vector{Int}
  SphereTree{N, T}() where {N, T <: AbstractFloat} = new{N, T}([], [], [], [])
end

export distance

function nearest_sphere(tree::SphereTree{N, T}, point::SVector{N, T})::Int where {N, T <: AbstractFloat}
  return argmin([sum((center .- point) .^ 2) - srad
                 for (center, srad) in zip(tree.centers, tree.squared_radii)])
end

function project(point::SVector{N, T}, sphere::Sphere{N, T})::SVector{N, T} where {N, T <: AbstractFloat}
  v = point .- sphere.center
  v /= norm(v)
  v = SVector{N, T}(v .* sphere.radius .+ sphere.center)
  draw!(v, color="orange")
  return v
end

function sample(bounds::Box{N, T})::SVector{N, T} where {N, T <: AbstractFloat}
  r = rand(N)
  x = SVector{N, T}((bounds.highs .- bounds.lows) .* r .+ bounds.lows)
  draw!(x, color="blue")
  return x
end

function push_sphere!(tree::SphereTree{N, T}, parent::Int, sphere::Sphere{N, T}) where {N, T <: AbstractFloat}
  child = 1 + length(tree.parent)
  push!(tree.centers, sphere.center)
  push!(tree.squared_radii, sphere.radius ^ 2)
  push!(tree.parent, parent)
  push!(tree.children, [])
  push!(tree.children[parent], child)
end

function get_sphere(tree::SphereTree{N, T}, sphere::Int)::Sphere{N, T} where {N, T <: AbstractFloat}
  return Sphere(tree.centers[sphere], sqrt(tree.squared_radii[sphere]))
end

function evt_step!(tree::SphereTree{N, T}, bounds::Box{N, T}, obstacles::Vector; min_radius=10.) where {N, T <: AbstractFloat}
  x = sample(bounds)
  x_near = nearest_sphere(tree, x)
  x_new = project(x, get_sphere(tree, x_near))
  x_new_rad = distance(x_new, obstacles)
  # Sphere is of minimum radius and center is in bounds.
  # Center can be out of bounds due to projection from inside sphere.
  if x_new_rad > min_radius && distance(x_new, bounds) < 0
    push_sphere!(tree, x_near, Sphere(x_new, x_new_rad))
  end
end

function evt(origin::SVector{N, T}, bounds::Box{N, T}, obstacles::Vector, steps::Int) where {N, T <: AbstractFloat}
  tree = SphereTree{N, T}()
  radius = distance(origin, obstacles)
  origin_sphere = Sphere(origin, radius)
  push_sphere!(tree, 1, origin_sphere)
  for iter in 1:steps
    evt_step!(tree, bounds, obstacles)
  end
  return tree
end

export evt

function get_path(tree::SphereTree{N, T}, last_idx::Int)::Vector{SVector{N, T}} where {N, T <: AbstractFloat}
  path = [tree.centers[last_idx]]
  while last_idx != 1
    last_idx = tree.parent[last_idx]
    pushfirst!(path, tree.centers[last_idx])
  end
  return path
end

function evt_goal(origin::SVector{N, T}, bounds::Box{N, T}, obstacles::Vector, goal::SVector{N, T}; min_radius=10.) where {N, T <: AbstractFloat}
  tree = SphereTree{N, T}()
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

function draw!(box::Box{2, T}; color="red") where {T <: AbstractFloat}
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

function draw!(sphere::Sphere{2, T}; points=64, color="red") where {T <: AbstractFloat}
  cx, cy = sphere.center
  r = sphere.radius
  t = range(0, 2pi, length=points)
  x = cx .+ r .* cos.(t)
  y = cy .+ r .* sin.(t)
  Makie.plot!(x, y, color=color)
end

function draw!(obstacle::SVector{2, T}; delta=0.1, color="red") where {T <: AbstractFloat}
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

function draw!(tree::SphereTree{N, T}; color="green") where {N, T <: AbstractFloat}
  for (center, srad) in zip(tree.centers, tree.squared_radii)
    draw!(Sphere(center, sqrt(srad)), color=color)
  end
end

export draw!

end # module
