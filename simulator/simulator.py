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

    def initialize(self, waypoints, dt = 1):
        self.obstacle = SimObstacle(waypoints, dt=dt)
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

    def initialize(self, trajecory, dt = 1):
        waypoints = trajecory[:, 0:2]
        headings = trajecory[:, 2]
        self.vehicle = SimVehicle(waypoints, headings, dt=dt)
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

    def begin(self, avPlanner):
        obstacleTrajectories = avPlanner.getObstacleTrajectories()
        obstacleTrajectories = []
        for avTrajectory in obstacleTrajectories:
            poseTable = avTrajectory.pose_table
            obstacleWaypoints = np.asarray([[pose.x, pose.y] for pose in poseTable])
            obstacleDt = avTrajectory.dt
            print(obstacleWaypoints)
            print(obstacleDt)
            print(simDt)
            obstacle = Obstacle()
            self.add_widget(obstacle)
            obstacle.initialize(obstacleWaypoints, obstacleDt)
            self.obstacles.append(obstacle)
            

        vehicleTrajectory = avPlanner.solveTrajectory()
        vehicleDt = vehicleTrajectory.dt
        vehicleWaypoints = np.asarray([[state.x, state.y, state.psi] for state in vehicleTrajectory.av_state_table])
        self.actorVehicle.initialize(vehicleWaypoints, vehicleDt)
        print(vehicleWaypoints)
        self.draw_trajectory()

    def draw_trajectory(self):
        for obstacle in self.obstacles:
            trajecory = obstacle.obstacle.get_trajectory()
            points = self.create_line_point(trajecory)
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

        # Initialize the simulator
        sim = Simulator()
        sim.begin(planner)

        # Update the simulator at the specified rate (simDt)
        Clock.schedule_interval(sim.update, simDt)
        return sim


if __name__ == '__main__':
    # Make sure that the simulator is not resizable
    Config.set('graphics', 'resizable', False)

    # Use user specified width and height as window width and height
    Window.size = (args.myWidth, args.myHeight)

    # Set the background of the simulator to white
    Window.clearcolor = (1, 1, 1, 1)

    # Begin the simulator app
    SimulatorApp().run()
