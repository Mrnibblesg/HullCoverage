# This file contains the code for each cleaning robot.
# I probably want to get rid of pymunk. I can do collision detection myself,
# since we're only really worried about collisions with the boundary
# and with other robots. We don't need a whole physics solver.
import pymunk
from math import pi
from surface import PARAMS

# Each robot has:
#   itself...
#       - pose
#   - an internal model of its environment
#   - sensors telling it where it is and how it's moving
#       - IMU
#       - barometer
#       - timer
# Each robot can:
#   - move to a specified goal based on its sensors and dead reckoning
#   - Keep track of where it's been
#   - Communicate with robots nearby,
#       exchanging info about what has been covered already.

# Simulation details:
# We'll have sensor noise be a toggle and start without sensor noise.


class Robot:
    def __init__(self):
        self.body = pymunk.Body(5, pymunk.moment_for_circle(1, 0, 30))
        self.body.position = (300, 50)
        self.shape = pymunk.Circle(self.body, 30)
        self.max_speed = 2
        self.turn_speed = 0.5

        self.baro = Barometer(self)
        self.IMU = IMU(self)
        self.motor = Motor(self)
        self.internal_model = InternalModel()

    # The job of the planner is to decide the next
    # goals of the robot. The ultimate goal is full coverage.
    # To this end, we update our coverage map, see if anyone is nearby,
    # and then optionally update our PID controllers using some sort
    # of path finding algorithm

    def tick(self):
        self.planner()  # Set target parameters and control signals
        self.communicate()
        self.move()  # Apply forces

    def planner(self):
        print('Update coverage map')
        self.internal_model.move()

        print('check for neighbors to exchange info')

        self.motor.angle_controller(pi)
        self.motor.velo_controller(0)

    def communicate(self):
        print("search for nearby bots")


# Base class for anything that is a sensor and can measure noise
class Sensor:
    accuracy = 1
    body = None

    def __init__(self, owner):
        self.body = owner


# In charge of the forces driving the robot, directly intertwined with the PIDs
class Motor(Sensor):
    max_power = 30  # units: ??
    rotation = 0
    forward = 0

    def __init__(self, owner):
        super().__init__(owner)
        self.rotation = 0
        self.forward = 0

    # Not a full PID yet
    # apply the full power to get the values to the goal
    def angle_controller(self, goal):
        self.rotation = 0  # TODO remove this patchwork when implementing PID
        diff = goal - self.owner.IMU.psi
        if (diff > 0):
            self.rotation = self.max_power
        else:
            self.rotation = -self.max_power

    def velo_controller(self, goal):
        print("control velocity")


# How to simulate the barometer?
# We can assume it's pretty accurate, with minimal amounts of noise in pressure
class Barometer(Sensor):
    def __init__(self, owner):
        super().__init__(owner)
        self.depth = 0  # meters

    def measure(self):
        self.depth = self.owner.body.position[1]


# Detects current acceleration (3-axis accelerometer/gyroscope)
# could you measure water flow for a more accurate reading of velocity?
# Better to use quaternions for this but it's euler angles for now
class IMU(Sensor):
    def __init__(self, owner):
        super().__init__(owner)
        # AKA roll, pitch, yaw
        self.phi = 0
        self.theta = 0
        self.psi = 0  # The only one useful for a flat surface

    def measure(self):
        print("IMU measurement of yaw")


# Contains the internal model, basically the bot knowing where it has been
# based on landmarks. This will be shared with neighboring bots
# whenever they are encountered.

# Internal models can be modeled by either:
#   A bunch of circles on a continuous space (Maybe this?)
#       How would I determine areas that aren't covered yet?
#       I can simply simulate this approach by using very small squares.
#       Not required at all, but save space by using adaptive LOD on quadtrees.
#   A matrix of grid cells, containing whether they're covered or not. (Meh..)
#       A graph of them would work better, especially for a
#       surface with both concave and convex regions. Though, Alli said that
#       there was a different approach that was better

class InternalModel:
    # Defines the size of an internal model square, in m.
    resolution = 0.1
    grid_width = int(PARAMS.SURFACE_DIMS_M[0] / resolution)
    grid_height = int(PARAMS.SURFACE_DIMS_M[1] / resolution)

    # The robot is pre-loaded with the shape of its environment
    # so it can model its cleaned area.
    space = [[False] * grid_width] * grid_height

    def __init__(self):
        self.color = "red"

    def merge(model):
        print("Merge data models here")

    # Tick the robot's position and update the internal model with new
    # spots cleaned.

    def move(self):
        print("Moving")
