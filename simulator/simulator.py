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

parser = argparse.ArgumentParser(description='Av Trajectory Planner Simulator')
parser.add_argument("-f", "--envFile", type=str, default="sample_trajectory.txt", help='json file that stores the environment variable that will be used to populate the planner')
args = parser.parse_args()

class Obstacle(Widget):
    obstacle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)
    point = []

    def initialize(self, waypoints, dt = 1):
        self.obstacle = SimObstacle(waypoints, dt=0.2)
        self.obstacle.create_trajectory(dt=dt)
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
        self.vehicle = SimVehicle(waypoints, headings, dt=0.05)
        self.vehicle.create_trajectory(dt=dt)
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
    # obstacle0 = ObjectProperty(None)
    obstacles = ListProperty(None)

    def create_line_point(self, trajectory):
        points = []
        for traj in trajectory:
            points += [float(traj[0]), float(traj[1])]
        return points

    def begin(self, avPlanner):
        obstacleTrajectories = avPlanner.getObstacleTrajectories()
        for avTrajectory in obstacleTrajectories:
            poseTable = avTrajectory.poseTable
            obstacleWaypoints = np.asarray([[pose.x, pose.y] for pose in poseTable])
            obstacleDt = avTrajectory.dt;
            obstacle = Obstacle()
            self.add_widget(obstacle)
            obstacle.initialize(obstacleWaypoints, obstacleDt)
            self.obstacles.append(obstacle)
            print(obstacleWaypoints)

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
        sim = Simulator()
        sim.begin(planner)
        Clock.schedule_interval(sim.update, 1.0/60.0)
        return sim


if __name__ == '__main__':
    planner = av.Planner()
    planner.loadFromJson(args.envFile)
    Config.set('graphics', 'resizable', False)
    Window.size = (500, 300)
    Window.clearcolor = (1, 1, 1, 1)
    SimulatorApp().run()
