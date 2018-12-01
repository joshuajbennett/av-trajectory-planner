from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line
from kivy.clock import Clock
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty, ListProperty
from kivy.vector import Vector
from simulatorObject import SimObstacle, SimVehicle
import numpy as np
from kivy.config import Config
from kivy.core.window import Window
import pdb
import sys
sys.path.append('../build')
import AvTrajectoryPlanner as av
import argparse

simDt = 1.0/30.0

parser = argparse.ArgumentParser(description='Av Trajectory Planner Simulator')
parser.add_argument("-e", "--envFile", type=str, default="sample_trajectory.txt", help='json file that stores the environment variable that will be used to populate the planner')
parser.add_argument("--myWidth", type=int, default=1000, help="The width of the simulator windows")
parser.add_argument("--myHeight", type=int, default=500, help="The height of the simulator windows")

args = parser.parse_args(sys.argv[2:])

class Obstacle(Widget):
    obstacle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)
    point = []

    def initialize(self, trajectory):
        self.obstacle = SimObstacle(trajectory)
        self.obstacle.create_trajectory(dt=simDt)
        self.trajGenerator = self.obstacle.trajectory_generator()
        start_pos = self.obstacle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        self.point = next(self.trajGenerator, self.point)
        self.pos = [float(self.point[0]) - 25, float(self.point[1]) - 12.5]
        self.radangle = float(self.point[2])
        self.angle = float(np.degrees(self.radangle))

class Vehicle(Widget):
    vehicle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)
    point = []

    def initialize(self, trajectory):
        self.vehicle = SimVehicle(trajectory)
        self.vehicle.create_trajectory(dt=simDt)
        self.trajGenerator = self.vehicle.trajectory_generator()
        start_pos = self.vehicle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        self.point = next(self.trajGenerator, self.point)
        self.pos = [float(self.point[0]) - 25, float(self.point[1]) - 12.5]
        self.radangle = float(self.point[2])
        self.angle = float(np.degrees(self.radangle))

class Simulator(Widget):
    actorVehicle = ObjectProperty(None)
    obstacles = ListProperty(None)

    def create_line_point(self, trajectory):
        points = []
        for traj in trajectory:
            points += [float(traj[0]), float(traj[1])]
        return points

    def begin(self, obstacleTrajectories, vehicleTrajectory):
        for trajectory in obstacleTrajectories:
            obstacle = Obstacle()
            self.add_widget(obstacle)
            obstacle.initialize(trajectory)
            self.obstacles.append(obstacle)

        self.actorVehicle.initialize(vehicleTrajectory)
        self.draw_trajectory()

    def draw_trajectory(self):
        for obstacle in self.obstacles:
            trajectory = obstacle.obstacle.get_trajectory()
            points = self.create_line_point(trajectory)
            print(points)
            with self.canvas:
                Color(1, 0, 1, 0.8)
                Line(points=points, width=1, dash_length=10, dash_offset=10)
        vehicleTrajectory = self.actorVehicle.vehicle.get_trajectory()
        points = self.create_line_point(vehicleTrajectory)
        with self.canvas:
            Color(0, 1, 0, 0.8)
            Line(points=points, width=1, dash_length=10, dash_offset=10)


    def update(self, dt):
        self.actorVehicle.move()

        for obstacle in self.obstacles:
            obstacle.move()


class SimulatorApp(App):

    def build(self):
        # Initialize Planner and load the environment through the envFile
        planner = av.Planner()
        with open(args.envFile) as file:
            envFileStr = file.read()
        planner.loadFromJson(envFileStr)

        # Plan the vehicle trajectory
        vehicleTrajectory = planner.solveTrajectory()

        # Get the obstacle trajectories
        obstacleTrajectories = planner.getObstacleTrajectories()

        # Get the max and min of the trajectories
        max_x, min_x, max_y, min_y = get_env_max_min(obstacleTrajectories, vehicleTrajectory)

        print(max_x, min_x, max_y, min_y)

        # Initialize the simulator
        sim = Simulator()
        sim.begin(obstacleTrajectories, vehicleTrajectory)

        # Update the simulator at the specified rate (simDt)
        Clock.schedule_interval(sim.update, simDt)
        return sim

def get_max_min_from_waypoints(waypoints):
    max_x = np.max(waypoints[:, 0])
    max_y = np.max(waypoints[:, 1])
    min_x = np.min(waypoints[:, 0])
    min_y = np.min(waypoints[:, 1])
    return max_x, min_x, max_y, min_y

def get_max_min_from_pose_table(pose_table):
    obstacleWaypoints = np.asarray([[pose.x, pose.y] for pose in pose_table])
    return get_max_min_from_waypoints(obstacleWaypoints)

def get_max_min_from_obstacle_trajectory(av_trajectory):
    pose_table = av_trajectory.table
    return get_max_min_from_pose_table(pose_table)

def get_max_min_from_obstacle_trajectories(obstacle_trajectories):
    max_x = -float("inf")
    max_y = -float("inf")
    min_x = float("inf")
    min_y = float("inf")
    for av_trajectory in obstacle_trajectories:
        if len(av_trajectory.table) > 0:
            raise Exception("Empty obstacle trajectory")
        curr_max_x, curr_min_x, curr_max_y, curr_min_y = get_max_min_from_obstacle_trajectory(av_trajectory)

        max_x = max(curr_max_x, max_x)
        max_y = max(curr_max_y, max_y)
        min_x = min(curr_min_x, min_x)
        min_y = min(curr_min_y, min_y)

    return max_x, min_x, max_y, min_y

def get_max_min_from_vehicle_trajectory(vehicle_trajectory):
    vehicle_waypoints = np.asarray([[state.x, state.y] for state in vehicle_trajectory.av_state_table])
    return get_max_min_from_waypoints(vehicle_waypoints)

def get_env_max_min(obstacle_trajectories, vehicle_trajectory):
    obstacle_max_min = get_max_min_from_obstacle_trajectories(obstacle_trajectories)
    vehicle_max_min = get_max_min_from_vehicle_trajectory(vehicle_trajectory)

    return max(vehicle_max_min[0], obstacle_max_min[0]), min(vehicle_max_min[1], obstacle_max_min[1]), max(vehicle_max_min[2], obstacle_max_min[2]), min(vehicle_max_min[3], obstacle_max_min[3])

if __name__ == '__main__':
    # Make sure that the simulator is not resizable
    Config.set('graphics', 'resizable', False)

    # Use user specified width and height as window width and height
    Window.size = (args.myWidth, args.myHeight)

    # Set the background of the simulator to white
    Window.clearcolor = (1, 1, 1, 1)

    # Begin the simulator app
    SimulatorApp().run()

