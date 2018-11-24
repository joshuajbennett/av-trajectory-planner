import numpy as np
import matplotlib.pyplot as plt

from simulatorObject import SimObstacle, SimVehicle


phi = np.linspace(0, 2.*np.pi, 40)
r = 0.5 + np.cos(phi)         # polar coords
X, Y = r * np.cos(phi), r * np.sin(phi)    # convert to cartesian

waypoints = np.stack([X,Y], -1)

obstacle = SimObstacle(waypoints)
obstacle.create_trajectory(dt=0.5)
obstacleWaypoints = obstacle.get_waypoints()
obstacleTrajectory = obstacle.get_trajectory()

arrowSize = 0.2

heading = obstacleTrajectory[:,2]
X, Y = obstacleTrajectory[:,0], obstacleTrajectory[:,1]
U, V = X + arrowSize*np.cos(heading), Y + arrowSize*np.sin(heading)

fig, ax = plt.subplots()
ax.plot(obstacleWaypoints[:,0], obstacleWaypoints[:,1], 'ro')
ax.plot(obstacleTrajectory[:,0], obstacleTrajectory[:,1], 'r-')
ax.plot((X, U), (Y, V), 'k-')
plt.show()

waypoints = np.stack([X,Y], -1)
vehicle = SimVehicle(waypoints, heading, dt=0.5)

vehicle.create_trajectory(dt=0.2)
vehicleWaypoints = vehicle.get_waypoints()
vehicleTrajectory = vehicle.get_trajectory()

arrowSize = 0.2

X, Y, heading = vehicleTrajectory[:,0], vehicleTrajectory[:,1], vehicleTrajectory[:,2]
U, V = X + arrowSize*np.cos(heading), Y + arrowSize*np.sin(heading)

fig, ax = plt.subplots()
ax.plot(vehicleWaypoints[:,0], vehicleWaypoints[:,1], 'ro')
ax.plot(vehicleTrajectory[:,0], vehicleTrajectory[:,1], 'r-')
ax.plot((X, U), (Y, V), 'k-')
plt.show()