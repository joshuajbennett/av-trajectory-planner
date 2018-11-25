from abc import ABC, abstractmethod
from scipy.interpolate import splprep, splev, interp1d
import numpy as np

class SimObject(ABC):
    """
    waypoints are nx2 array that contains the points where the
    object must pass through
    """
    def __init__(self, waypoints=None, dt=1):
        super().__init__()
        self.waypoints = waypoints
        self.dt = dt
        self.trajectory = None

    @abstractmethod
    def create_trajectory(self, dt):
        pass

    def get_start_point(self):
        if np.any(self.waypoints):
            return self.waypoints[0]
        return None
    
    def get_goal_point(self):
        if np.any(self.waypoints):
            return self.waypoints[-1]
        return None

    def get_waypoints(self):
        return self.waypoints
    
    def set_waypoints(self, waypoints):
        self.waypoints = waypoints

    def get_trajectory(self):
        return self.trajectory

    def set_trajectory(self, trajectory):
        self.trajectory = trajectory

    def trajectory_generator(self, dt=None):
        for point in self.trajectory:
            yield point
    

class SimObstacle(SimObject):
    def __init__(self, waypoints, dt=1):
        super(SimObstacle, self).__init__(waypoints, dt)

    def create_trajectory(self, dt=None):
        if not dt:
            dt = self.dt
        num = int(self.dt/dt * len(self.waypoints))
        # print(num)
        x = self.waypoints[:,0]
        y = self.waypoints[:,1]
        tck, u = splprep([x, y], s=0)
        u = np.linspace(u.min(), u.max(), num)
        x, y = splev(u, tck)
        xp, yp = splev(u, tck, der=1)
        heading = np.arctan2(yp, xp)
        self.trajectory = np.stack([x, y, heading], -1)

        

class SimVehicle(SimObject):
    def __init__(self, waypoints, headings, dt=1):
        super(SimVehicle, self).__init__(waypoints, dt)
        self.headings = headings

    def create_trajectory(self, dt=None):
        if not dt:
            dt = self.dt
        num = int(self.dt/dt * len(self.waypoints))
        # print(num)
        x = self.waypoints[:,0]
        y = self.waypoints[:,1]
        tck, u = splprep([x, y], s=0)
        u_new = np.linspace(u.min(), u.max(), num)
        x, y = splev(u_new, tck)
        unwrapHeading = np.unwrap(self.headings)
        interpedHeading = np.interp(u_new, u, unwrapHeading)
        interpedHeading = ((interpedHeading + np.pi)%(2*np.pi))-np.pi
        self.trajectory = np.stack([x, y, interpedHeading], -1)