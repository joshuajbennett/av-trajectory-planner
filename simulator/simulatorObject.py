from abc import ABC, abstractmethod
from scipy.interpolate import splprep, splev, interp1d
import math
import numpy as np
import pdb

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
        start_point = self.planner_trajectory.table[0]
        return np.asarray([start_point.x, start_point.y, start_point.psi])

    def get_goal_point(self):
        start_point = self.planner_trajectory.table[-1]
        return np.asarray([start_point.x, start_point.y, start_point.psi])

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
            dt = self.planner_trajectory.dt
        num = math.ceil(self.planner_trajectory.dt/dt * len(self.planner_trajectory.table)) + 1

        self.trajectory = []

        time = 0
        for i in range(num):
            interp = self.planner_trajectory.interpolate(time)
            self.trajectory.append([interp.x, interp.y, interp.psi])
            time += dt


class SimVehicle(SimObject):
    def __init__(self, planner_trajectory):
        super(SimVehicle, self).__init__(planner_trajectory)

    def create_trajectory(self, dt=None):
        if not dt:
            dt = self.planner_trajectory.dt
        num = math.ceil(self.planner_trajectory.dt/dt * len(self.planner_trajectory.table)) + 1

        self.trajectory = []

        time = 0
        for i in range(num):
            interp = self.planner_trajectory.interpolate(time)
            self.trajectory.append([interp.x, interp.y, interp.psi])
            time += dt
