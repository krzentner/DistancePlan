module DistancePlan

function norm(v::Array{T, N})::T where {T <: AbstractFloat, N}
  return sqrt(sum(v .^ 2))
end

function distance(center::Array{T, N}, obstacle::Array{T, N})::T where {T <: AbstractFloat, N}
  return norm(center .- obstacle)
end

abstract type Volume end

struct Box{T <: Number, N} <: Volume
    lows::Array{T, N}
    highs::Array{T, N}
end

function distance(center::Array{T, N}, obstacle::Box{T, N})::T where {T <: AbstractFloat, N}
    diffs = [(obstacle.lows .- center) (center .- obstacle.highs)]
    max_diffs = maximum(diffs, dims=2)
    return norm(max_diffs)
end

struct Sphere{T <: Number, N} <: Volume
    center::Array{T, N}
    radius::T
end

function distance(center::Array{T, N}, obstacle::Sphere{T, N})::T where {T <: AbstractFloat, N}
    return norm(center .- obstacle.center) - obstacle.radius
end

function distance(center::Array{T, N}, obstacle::AbstractVector)::T where {T <: AbstractFloat, N}
    return minimum([distance(center, obs) for obs=obstacle])
end

struct SphereTree{T <: Number, N}
    centers::Vector{Array{T, N}}
    squared_radii::Vector{Array{T, N}}
    children::Vector{Vector{UInt}}
    parent::Vector{UInt}
end

function nearest_sphere(tree::SphereTree{T, N}, point::Array{T, N})::UInt where {T <: AbstractFloat, N}
    return argmin(sum((point .- centers) .^ 2, dims=2) - tree.squared_radii)
end

function project(point::Array{T, N}, sphere::Sphere{T, N})::Array{T, N} where {T <: AbstractFloat, N}
  v = point .- sphere.center
  v /= norm(v)
  return v .* sphere.radius
end

function sample(bounds::Box{T, N})::Array{T, N} where {T <: AbstractFloat, N}
  r = rand(length(bounds.lows))
  return (bounds.highs .- bounds.lows) .* r .+ bounds.low
end

function push_sphere!(tree::SphereTree{T, N}, parent::UInt, sphere::Sphere{T, N}) where {T <: AbstractFloat, N}
  child = length(tree.parent)
  push!(tree.centers, sphere.center)
  push!(tree.squared_radii, sphere.radius ^ 2)
  push!(tree.parent, parent)
  push!(tree.children[parent], child)
end

function get_sphere(tree::SphereTree{T, N}, sphere::UInt)::Sphere{T, N} where {T <: AbstractFloat, N}
  return Sphere(tree.centers[sphere], sqrt(tree.squared_radii[sphere]))
end

function evt_step!(tree::SphereTree{T, N}, bounds::Box{T, N}, obstacles::Vector) where {T <: AbstractFloat, N}
  x = sample(bounds)
  x_near = nearest_sphere(tree, point)
  x_new = project(x, get_sphere(tree, x_near))
  x_new_rad = distance(x_new, obstacles)
  push_sphere!(tree, x_near, Sphere(x_new, x_new_rad))
end

end # module
