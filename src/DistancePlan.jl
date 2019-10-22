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
    centers::Array{Float64}
    squared_radii:Array{Float64}
    down::Vector{Vector{UInt}}
    up::Vector{UInt}
end

function nearest_sphere(tree::SphereTree, point::Vector{Float64})::UInt
    return argmin(sum((point .- centers) .^ 2, dims=2) - tree.squared_radii)
end

end # module
