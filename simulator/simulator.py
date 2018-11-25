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

dt = 1.0/60.0

class Obstacle(Widget):
    obstacle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)

    def initialize(self, waypoints):
        self.obstacle = SimObstacle(waypoints, dt=0.2)
        self.obstacle.create_trajectory(dt=dt)
        self.trajGenerator = self.obstacle.trajectory_generator()
        start_pos = self.obstacle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        point = next(self.trajGenerator, self.pos + [self.radangle])
        self.pos = [float(point[0])-25, float(point[1])-12.5]
        self.radangle = float(point[2])
        self.angle = float(np.degrees(self.radangle))

class Vehicle(Widget):
    vehicle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)

    def initialize(self, trajecory):
        waypoints = trajecory[:, 0:2]
        headings = trajecory[:, 2]
        self.vehicle = SimVehicle(waypoints, headings, dt=0.2)
        self.vehicle.create_trajectory(dt=dt)
        self.trajGenerator = self.vehicle.trajectory_generator()
        start_pos = self.vehicle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        point = next(self.trajGenerator, self.pos + [self.radangle])
        self.pos = [float(point[0]) - 25, float(point[1]) - 12.5]
        self.radangle = float(point[2])
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

    def begin(self, vehicleTrajectoryFile, obstacleWaypointsFileList):
        for obstacleWaypointsFile in obstacleWaypointsFileList:
            obstacleWaypoints = np.loadtxt(obstacleWaypointsFile)
            obstacle = Obstacle()
            self.add_widget(obstacle)
            obstacle.initialize(obstacleWaypoints)
            self.obstacles.append(obstacle)
        
        # self.add_widget(self.obstacles)

        vehicleTrajectory = np.loadtxt(vehicleTrajectoryFile)
        self.actorVehicle.initialize(vehicleTrajectory)

        self.draw_trajectory()
    
    def draw_trajectory(self):
        for obstacle in self.obstacles:
            trajecory = obstacle.obstacle.get_trajectory()
            points = self.create_line_point(trajecory)
            print(points)
            with self.canvas:
                Color(1, 0, 1, 0.8)
                Line(points=points, width=2)

        
    def update(self, dt):
        self.actorVehicle.move()

        for obstacle in self.obstacles:
            obstacle.move()


class SimulatorApp(App):

    def build(self):
        sim = Simulator()
        sim.begin("vehicleTrajectory.txt", ["obstacleWaypoints.txt", "obstacleWaypoints1.txt"])
        Clock.schedule_interval(sim.update, dt)
        return sim


if __name__ == '__main__':
    Config.set('graphics', 'resizable', False)
    Window.size = (500, 300)
    Window.clearcolor = (1, 1, 1, 1)
    SimulatorApp().run()