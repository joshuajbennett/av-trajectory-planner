import sys
sys.path.append('build')
import AvTrajectoryPlanner as av
import math
import pdb

planner = av.Planner()
planner.setSolverMaxTime(10.0)
planner.setSolverTimeStep(0.1)
planner.setInitialState(av.AvState())

goal = av.AvState()
goal.x = 5
goal.y = 1
planner.setGoal(goal)

obstacle = av.ObstacleStatic()

outline = av.Boundary()
point = av.Point()
point.x = 0.5
point.y = 0.5
outline.vertices.append(point)
point.x = -0.5
outline.vertices.append(point)
point.y = -0.5
outline.vertices.append(point)
point.y = 0.5
outline.vertices.append(point)
obstacle.obs_outline = outline

pose = av.Pose()
pose.x = 3
pose.y = 0.5
pose.theta = 0.8
obstacle.obs_pose = pose

planner.appendObstacleStatic(obstacle)

traj = planner.solveTrajectory()

pdb.set_trace()


