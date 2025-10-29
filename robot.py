# This file contains the code for each cleaning robot.
# I probably want to get rid of pymunk. I can do collision detection myself,
# since we're only really worried about collisions with the boundary
# and with other robots. We don't need a whole physics solver.
import pymunk

# Each robot has:
#   itself...
#       - pose
#   - an internal model of its environment
#   - sensors telling it where it is and how it's moving
#       - IMU
#       - barometer
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
        self.internal_model = InternalModel()

    # The job of the planner is to decide the next
    # goals of the robot. The ultimate goal is full coverage.
    # To this end, we update our coverage map, see if anyone is nearby,
    # and then optionally update our PID controllers using some sort
    # of path finding algorithm
    def planner():
        print('Update coverage map')
        print('check for neighbors to exchange info')
        print('update PIDs')

    def angle_PID():
        print('turn now')

    def velo_PID():
        print('velocity')


# Base class for anything that is a sensor and can measure noise
class Sensor:
    accuracy = 1
    body = None

    def __init__(self, owner):
        self.body = owner


# How to simulate the barometer?
# We can assume it's pretty accurate, with minimal amounts of noise in pressure
class Barometer(Sensor):
    def __init__(self, body):
        self.depth = 0  # meters


# Detects current acceleration (3-axis accelerometer/gyroscope)
class IMU(Sensor):
    def __init__(self, body):
        self.phi = 0
        self.theta = 0
        self.psi = 0


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
    resolution = 100
    space = [[False] * resolution] * resolution

    def __init__(self):
        self.color = "red"  # used for visualizing a robot's internal model

    def merge(model):
        print("Merge data models here")
