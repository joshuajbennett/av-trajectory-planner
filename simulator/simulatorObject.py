from abc import ABC, abstractmethod
from scipy.interpolate import splprep, splev, interp1d
import numpy as np

class SimObject(ABC):
    """
    waypoints are nx2 array that contains the points where the
    object must pass through
    """
    def __init__(self, planner_trajectory):
        super().__init__()
        self.planner_trajectory = planner_trajectory
        self.trajectory = None

    @abstractmethod
    def create_trajectory(self, dt):
        pass

    def get_start_point(self):
        return self.planner_trajectory['table'][0]

    def get_goal_point(self):
        return self.planner_trajectory['table'][-1]

    def get_trajectory(self):
        return self.trajectory

    def set_trajectory(self, trajectory):
        self.trajectory = trajectory

    def trajectory_generator(self, dt=None):
        for point in self.trajectory:
            yield point


class SimObstacle(SimObject):
    def __init__(self, planner_trajectory):
        super(SimObstacle, self).__init__(planner_trajectory)

    def create_trajectory(self, dt=None):
        if not dt:
            dt = self.planner_trajectory['dt']
        num = int(self.dt/self.planner_trajectory['dt'] * len(self.planner_trajectory['table']))

        self.trajectory = np.zeros([num, 3], np.float32)

        time = 0
        for i in range(num):
            interp = self.planner_trajectory.interpolate(time)
            self.trajectory[time,:] = np.asarray([interp.x, interp.y, interp.psi])


class SimVehicle(SimObject):
    def __init__(self, planner_trajectory):
        super(SimVehicle, self).__init__(planner_trajectory)

    def create_trajectory(self, dt=None):
        if not dt:
            dt = self.planner_trajectory['dt']
        num = int(self.dt/self.planner_trajectory['dt'] * len(self.planner_trajectory['table']))

        self.trajectory = np.zeros([num, 3], np.float32)

        time = 0
        for i in range(num):
            interp = self.planner_trajectory.interpolate(time)
            self.trajectory[time,:] = np.asarray([interp.x, interp.y, interp.psi])
