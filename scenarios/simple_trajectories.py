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
