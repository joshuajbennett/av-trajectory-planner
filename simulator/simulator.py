from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse
from kivy.clock import Clock
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from simulatorObject import SimObstacle, SimVehicle
import numpy as np
from kivy.config import Config

dt = 1.0/60.0

class Obstacle(Widget):
    obstacle = None
    trajGenerator = None
    angle = NumericProperty(0)
    radangle = NumericProperty(0)

    def initialize(self, waypoints):
        self.obstacle = SimObstacle(waypoints)
        self.obstacle.create_trajectory()
        self.trajGenerator = self.obstacle.trajectory_generator()
        start_pos = self.obstacle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        point = next(self.trajGenerator, self.pos + [self.radangle])
        self.pos = [float(point[0]), float(point[1])]
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
        self.vehicle = SimVehicle(waypoints, headings)
        self.vehicle.create_trajectory()
        self.trajGenerator = self.vehicle.trajectory_generator()
        start_pos = self.vehicle.get_start_point()
        self.pos = [float(start_pos[0]), float(start_pos[1])]

    def move(self):
        point = next(self.trajGenerator, self.pos + [self.radangle])
        self.pos = [float(point[0]), float(point[1])]
        self.radangle = float(point[2])
        self.angle = float(np.degrees(self.radangle))

class Simulator(Widget):
    actorVehicle = ObjectProperty(None)
    obstacle0 = ObjectProperty(None)

    def begin(self, vehicleTrajectoryFile, obstacleWaypointsFileList):
        obstacleWaypoints = np.loadtxt(obstacleWaypointsFileList[0])
        self.obstacle0.initialize(obstacleWaypoints)

        vehicleTrajectory = np.loadtxt(vehicleTrajectoryFile)
        self.actorVehicle.initialize(vehicleTrajectory)

    def update(self, dt):
        self.actorVehicle.move()
        self.obstacle0.move()


class SimulatorApp(App):

    def build(self):
        sim = Simulator()
        sim.begin("vehicleTrajectory.txt", ["obstacleWaypoints.txt"])
        Clock.schedule_interval(sim.update, dt)
        return sim


if __name__ == '__main__':
    Config.set('graphics', 'resizable', False)
    SimulatorApp().run()
