import sys
sys.path.append('build')
import AvTrajectoryPlanner as av
import math

planner = av.Planner(av.AvState(0,0,0,0,0), av.AvState(5,1,0,0,0), av.AvParams(1.0,1.0,0.5,4), av.Boundary([av.Point(0.5,0.5), av.Point(-0.5, 0.5), av.Point(-0.5, -0.5), av.Point(-0.5, 0.5)]), 10, 0.1)

obstacle = av.ObstacleStatic()
obstacle.outline = av.Boundary([av.Point(0.5,0.2), av.Point(-0.5, 0.2), av.Point(-0.5, -0.2), av.Point(-0.5, 0.2)])
obstacle.obs_pose = av.Pose(3,0.5, 0.8)

planner.appendObstacleStatic(obstacle)

json = planner.saveToJson()

output = open("scenarios/simple_trajectory_1.txt","w")
output.write(json)
