import sys
sys.path.append('build')
import AvTrajectoryPlanner as av
import math
import pdb

planner = av.Planner(av.AvState(0,0,0,0,0), av.AvState(5,1,0,0,0), av.AvParams(1,1,1,5), av.Boundary([av.Point(0.5,0.5), av.Point(-0.5, 0.5), av.Point(-0.5, -0.5), av.Point(-0.5, 0.5)]), 10, 0.1)


obstacle = av.ObstacleStatic()
obstacle.obs_outline = av.Boundary([av.Point(0.5,0.2), av.Point(-0.5, 0.2), av.Point(-0.5, -0.2), av.Point(-0.5, 0.2)])
obstacle.obs_pose = av.Pose(3,0.5, 0.8)

planner.appendObstacleStatic(obstacle)

traj = planner.solveTrajectory()

pdb.set_trace()


