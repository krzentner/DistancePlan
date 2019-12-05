import Pkg
Pkg.activate(".")
using DistancePlan
using Test
using StaticArrays

@testset "DistancePlan.jl" begin
    @test distance(SVector{2}([0. 0.]), SVector{2}([1. 1.])) == sqrt(2)
    @test distance(SVector{2}([0. 0.]), Box(SVector{2}([1., 1.]), SVector{2}([2., 2.]))) <= sqrt(2)
    @test distance(SVector{2}([0. 0.]), Sphere(SVector{2}([1., 1.]), 0.5)) == sqrt(2) - 0.5
    @test distance(SVector{2}([0. 0.]), [SVector{2}([0. 0.]), SVector{2}([1. 1.])]) == 0.
    @test distance(SVector{2}([0. 0.]), Box(SVector{2}([-1., -1.]), SVector{2}([1., 1.]))) < 0
    @test distance(SVector{2}([-2. 0.]), Box(SVector{2}([-1., -1.]), SVector{2}([1., 1.]))) == 1.
    @test distance(SVector{2}([2. 0.]), Box(SVector{2}([-1., -1.]), SVector{2}([1., 1.]))) == 1.
    @test distance(SVector{2}([0. 2.]), Box(SVector{2}([-1., -1.]), SVector{2}([1., 1.]))) == 1.
    @test distance(SVector{2}([0. -2.]), Box(SVector{2}([-1., -1.]), SVector{2}([1., 1.]))) == 1.
    @test distance(SVector{2}([0. 0.]), [SVector{2}([1. 1.]), SVector{2}([-1. -1.])]) == sqrt(2)
    # Write your own tests here.
end
