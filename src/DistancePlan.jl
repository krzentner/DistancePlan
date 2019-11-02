module DistancePlan

norm(v)::Float64 = sqrt(sum(v .^ 2))

distance(center, obstacle::Vector{Float64})::Float64 = norm(center .- obstacle)

struct Box
    lows::Vector{Float64}
    highs::Vector{Float64}
end

function distance(center, obstacle::Box)::Float64
    diffs = [(obstacle.lows .- center) (center .- obstacle.highs)]
    max_diffs = maximum(diffs, dims=2)
    return norm(max_diffs)
end

struct Sphere
    center::Vector{Float64}
    radius::Float64
end

function distance(center, obstacle::Sphere)::Float64
    return norm(center .- obstacle.center) - obstacle.radius
end

function distance(center, obstacle::Vector)::Float64
    return minimum([distance(center, obs) for obs=obstacle])
end

struct SphereTree
    centers::Vector{Vector{Float64}}
    squared_radii::Vector{Vector{Float64}}
    children::Vector{Vector{UInt}}
    parent::Vector{UInt}
end

function nearest_sphere(tree::SphereTree, point::Vector{Float64})::UInt
    return argmin(sum((point .- centers) .^ 2, dims=2) - tree.squared_radii)
end

function project(point::Vector{Float64}, sphere::Sphere)::Vector{Float64}
  v = point .- sphere.center
  v /= norm(v)
  return v .* sphere.radius
end

function sample(bounds::Box)::Vector{Float64}
  r = rand(length(bounds.lows))
  return (bounds.highs .- bounds.lows) .* r .+ bounds.low
end

function push_sphere!(tree::SphereTree, parent::UInt, sphere::Sphere)
  child = length(tree.parent)
  push!(tree.centers, sphere.center)
  push!(tree.squared_radii, sphere.radius ^ 2)
  push!(tree.parent, parent)
  push!(tree.children[parent], child)
end

function get_sphere(tree::SphereTree, sphere::UInt)::Sphere
  return Sphere(tree.centers[sphere], sqrt(tree.squared_radii[sphere]))
end

function evt_step!(tree::SphereTree, bounds::Box, obstacles::Vector)
  x = sample(bounds)
  x_near = nearest_sphere(tree, point)
  x_new = project(x, get_sphere(tree, x_near))
  x_new_rad = distance(x_new, obstacles)
  push_sphere!(tree, x_near, Sphere(x_new, x_new_rad))
end

end # module
