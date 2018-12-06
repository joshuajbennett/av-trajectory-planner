import sys
sys.path.append('build')
import AvTrajectoryPlanner as av
import math

planner = av.Planner(av.AvState(0,0,0,0,0), av.AvState(5,1,0,0,0), av.AvParams(1.0,1.0,0.5,4, 3), av.Boundary([av.Point(0.5,0.5), av.Point(-0.5, 0.5), av.Point(-0.5, -0.5), av.Point(0.5, -0.5)]), av.SolverParams(6, 0.01, 0.1, 3, True, True))

obstacle = av.ObstacleStatic()
obstacle.outline = av.Boundary([av.Point(0.5,0.2), av.Point(-0.5, 0.2), av.Point(-0.5, -0.2), av.Point(0.5, -0.2)])
obstacle.obs_pose = av.Pose(3,0.5, 0.8)

planner.appendObstacleStatic(obstacle)

dynamic_traj = av.ObstacleTrajectory()
dynamic_traj.dt = 2
dynamic_traj.outline = av.Boundary([av.Point(0.5,0.5), av.Point(-0.5, 0.5), av.Point(-0.5, -0.5), av.Point(0.5, -0.5)])
new_table = []
new_table.append(av.Pose(0,3,0))
new_table.append(av.Pose(3,3,0))
new_table.append(av.Pose(3,0,0))
new_table.append(av.Pose(0,0,0))
dynamic_traj.table = new_table

planner.appendObstacleTrajectory(dynamic_traj)

json = planner.saveToJson()

output = open("scenarios/simple_trajectory_1.txt","w")
output.write(json)


# This one is meant to be a really nice trajectory
car_boundary = av.Boundary([av.Point(2.5,0.7), av.Point(-0.7, 0.7), av.Point(-0.7, -0.7), av.Point(2.5, -0.7)])
planner = av.Planner(av.AvState(0,0,0,0,0), av.AvState(15,2,0.3,0,0), av.AvParams(1.0,1.0,0.5,4, 3), car_boundary , av.SolverParams(6, 0.01, 0.1, 3, False, False))

obstacle = av.ObstacleStatic()
obstacle.outline = car_boundary
obstacle.obs_pose = av.Pose(6,-1.0, 0.0)
planner.appendObstacleStatic(obstacle)

obstacle = av.ObstacleStatic()
obstacle.outline = car_boundary
obstacle.obs_pose = av.Pose(11,-1.0, 0.0)
planner.appendObstacleStatic(obstacle)

obstacle = av.ObstacleStatic()
obstacle.outline = car_boundary
obstacle.obs_pose = av.Pose(16,-0.5, 0.3)
planner.appendObstacleStatic(obstacle)

dynamic_traj = av.ObstacleTrajectory()
dynamic_traj.dt = 4
dynamic_traj.outline = car_boundary
new_table = []
new_table.append(av.Pose(-4,1,0))
new_table.append(av.Pose(10,1,0))
dynamic_traj.table = new_table
planner.appendObstacleTrajectory(dynamic_traj)



# dynamic_traj = av.ObstacleTrajectory()
# dynamic_traj.dt = 2
# dynamic_traj.outline = car_boundary
# new_table = []
# new_table.append(av.Pose(0,3,0))
# new_table.append(av.Pose(3,3,0))
# new_table.append(av.Pose(3,0,0))
# new_table.append(av.Pose(0,0,0))
# dynamic_traj.table = new_table
#
# planner.appendObstacleTrajectory(dynamic_traj)

json = planner.saveToJson()

output = open("simulator/sample_trajectory.txt","w")
output.write(json)


