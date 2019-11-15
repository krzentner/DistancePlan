using PyCall
anki_vector = pyimport("anki_vector")
util = pyimport("anki_vector.util")

args = anki_vector.util.parse_command_args()
@pywith anki_vector.Robot(args.serial) as robot begin
  robot.behavior.drive_off_charger()

  import Pkg
  Pkg.activate(".")

  using DistancePlan
  # include("../src/GraphDistancePlan.jl")
  # using GraphDistancePlan
  import Makie
  using StaticArrays

  scene = Makie.Scene()

  robot_theta = pi / 4
  robot_x = -250.
  robot_y = -250.

  B = 300.

  obstacles = [
      Box(SVector{2}([-150. -150.]), SVector{2}([150. 150.])),

      Box(SVector{2}([-B -B]), SVector{2}([-B B])), # Left
      Box(SVector{2}([B -B]), SVector{2}([B B])), # Right
      Box(SVector{2}([-B B]), SVector{2}([B B])), # Top
      Box(SVector{2}([-B -B]), SVector{2}([B -B])), # Bottom
     ]
  bounds = Box(SVector{2}([-B -B]), SVector{2}([B B]))
  goal = SVector{2}([250. 250.])
  origin = SVector{2}([robot_x robot_y])

  println("Planning...")
  tree, path = evt_goal(origin, bounds, obstacles, goal, min_radius=30.)
  push!(path, goal)
  println("Planning complete!")
  println("path = ", path)

  DistancePlan.draw!(origin)
  DistancePlan.draw!(bounds)
  DistancePlan.draw!(tree)
  DistancePlan.draw!.(obstacles)
  for point in path
    DistancePlan.draw!(point, color="purple")
  end
  Makie.plot!([x for (x, y) in path], [y for (x, y) in path], color="black")

  Makie.save("drive_vector.png", scene, resolution=(1000, 1000))

  for (x, y) in path
    println("going to (", x, ", ", y, ")")
    dx = x - robot_x
    dy = y - robot_y
    ego_theta = mod2pi(atan(dy, dx) - robot_theta)
    if ego_theta > pi
      ego_theta = ego_theta - 2pi
    end
    println("turning ", ego_theta, "rads")
    robot.behavior.turn_in_place(util.radians(ego_theta))
    println("driving ", sqrt(dx ^ 2 + dy ^ 2), "mm")
    robot.behavior.drive_straight(util.distance_mm(sqrt(dx ^ 2 + dy ^ 2)),
                                  util.speed_mmps(50))
    robot_x = x
    robot_y = y
    robot_theta = mod2pi(robot_theta + ego_theta)
    println("now at (", robot_x, ", ", robot_y, ")")
  end
end
