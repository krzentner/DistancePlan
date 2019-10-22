using DistancePlan
using Test

@testset "DistancePlan.jl" begin
    @test DistancePlan.distance([0., 0.], [1., 1.]) == sqrt(2)
    @test DistancePlan.distance([0., 0.],
                                DistancePlan.Box([1., 1.], [2., 2.])) == sqrt(2)
    @test DistancePlan.distance([0., 0.],
                                DistancePlan.Sphere([1., 1.], 0.5)) == sqrt(2) - 0.5
    @test DistancePlan.distance([0., 0.], [[0., 0.], [1., 1.]]) == 0.
    # Write your own tests here.
end
