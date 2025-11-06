# This file contains the code for each cleaning robot.
# I probably want to get rid of pymunk. I can do collision detection myself,
# since we're only really worried about collisions with the boundary
# and with other robots. We don't need a whole physics solver.
import pymunk
import numpy as np
from pymunk.vec2d import Vec2d
import math
from params import PARAMS


class Robot:
    radius = 1
    world = None

    def __init__(self, position):
        self.body = pymunk.Body(5, pymunk.moment_for_circle(
            1, 0, self.radius))

        self.body.position = position * PARAMS.PX_PER_M
        self.shape = pymunk.Circle(self.body, self.radius * PARAMS.PX_PER_M)

        self.baro = Barometer(self)
        self.IMU = IMU(self)
        self.motor = Motor(self)
        self.internal_model = InternalModel(self.body.position)

    # The job of the planner is to decide the next
    # goals of the robot. The ultimate goal is full coverage.
    # To this end, we update our coverage map, see if anyone is nearby,
    # and then optionally update our PID controllers using some sort
    # of path finding algorithm

    def tick(self, screen, pygame):
        print("Position: ", self.body.position)
        self.planner(screen, pygame)  # Set target parameters and control signals
        self.communicate()
        self.move()  # Apply forces from motors

    def planner(self, screen, pygame):
        print('Update coverage map')
        self.internal_model._update_ground_truth(self, screen, pygame)

        print('check for neighbors to exchange info')

        # Change motor signals
        self.motor.angle_controller(self.internal_model.psi_pred, math.pi)
        self.motor.velo_controller(0, 0)

    def communicate(self):
        pass

    def move(self):
        direction = self.body.angle
        force_vec = Vec2d(math.cos(direction), math.sin(direction))

        self.body.apply_force_at_local_point(force_vec * self.motor.forward, (0, 0))

        # TODO Alternatively, set the torque
        perp_offset = force_vec.perpendicular_normal()
        self.body.apply_force_at_local_point(force_vec * self.motor.rotation,
                                             perp_offset / PARAMS.PX_PER_M)
        self.body.apply_force_at_local_point(-force_vec * self.motor.rotation,
                                             -perp_offset / PARAMS.PX_PER_M)

        # TODO After our forces are applied, our sensors react to the changes.
        self.IMU.react()

    def visualize(self, screen, pygame):
        self.internal_model.visualize(screen, pygame)


# Base class for anything that is a sensor and can measure noise
class Sensor:
    accuracy = 1
    body = None

    def __init__(self, owner):
        self.owner = owner


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
    def angle_controller(self, current, goal):
        # print("Current: ", current)
        # print("Goal: ", goal)
        self.rotation = 0  # TODO remove this patchwork when implementing PID
        diff = goal - current
        if (diff > 0):
            self.rotation = self.max_power
        else:
            self.rotation = -self.max_power
        self.rotation = 0

    def velo_controller(self, current, goal):
        self.forward = self.max_power * 50


# How to simulate the barometer?
# We can assume it's pretty accurate, with minimal amounts of noise in pressure
class Barometer(Sensor):
    def __init__(self, owner):
        super().__init__(owner)
        self.depth = 0  # meters

    def measure(self):
        self.depth = self.owner.body.position[1]


# Detects current acceleration (3-axis accelerometer/gyroscope)
# TODO could you measure water flow for a more accurate reading of velocity?
# TODO Better to use quaternions for this but it's euler angles for now
class IMU(Sensor):
    def __init__(self, owner):
        super().__init__(owner)
        self.last_measurement = -1
        self.last_d_psi = 0
        self.last_d_forward = 0
        self.last_d_lateral = 0

        self.a_psi = 0
        self.a_forward = 0
        self.a_lateral = 0

    # Detect changes in acceleration. We get to hook into ground truth values
    # because we are simulating our sensors measuring the world.
    # Needs time diff between last tick and velocity diff from last tick
    def react(self):
        body = self.owner.body
        d_time = Robot.world.simulation_time - self.last_measurement

        velocity = body.velocity_at_world_point(body.position)
        ang_vel = body.angular_velocity
        psi = body.angle

        accel_forward = None
        accel_lateral = None
        accel_psi = None

        # Could I just use F=MA?
        accel_psi = (ang_vel - self.last_d_psi) / d_time
        self.last_d_psi = ang_vel

        # Use the rotation matrix to translate world-frame velocity to
        # robot-frame components for forward and lateral velocity.
        forward_vel = (velocity[0] * math.cos(psi)) + \
                      (velocity[1] * math.sin(psi))
        lat_vel = (-velocity[0] * math.sin(psi)) + \
                  (velocity[1] * math.cos(psi))

        accel_forward = (forward_vel - self.last_d_forward) / d_time
        self.last_d_forward = forward_vel

        accel_lateral = (lat_vel - self.last_d_lateral) / d_time
        self.last_d_lateral = lat_vel

        print("Lateral Acceleration: ", accel_lateral)
        print("Forward Acceleration: ", accel_forward)
        print("Radial Acceleration: ", accel_psi)

        self.a_psi = accel_psi
        self.a_forward = accel_forward
        self.a_lateral = accel_lateral

        self.last_measurement = Robot.world.simulation_time


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
    # Defines the size of an internal model square, in m, and in px.
    RES = 0.5
    GRID_BOX_PX = RES * PARAMS.PX_PER_M

    GRID_WIDTH = int(PARAMS.SURFACE_DIMS_M[0] / RES)
    GRID_HEIGHT = int(PARAMS.SURFACE_DIMS_M[1] / RES)

    # The robot is pre-loaded with the shape of its environment
    # so it can model its cleaned area.

    def __init__(self, position):
        # AKA roll, pitch, yaw
        self.phi_pred = 0
        self.theta_pred = 0
        self.psi_pred = 0  # The only one useful for a flat surface

        # Start out with knowledge. Predict the rest
        self.x_pred = position[0]
        self.y_pred = position[1]

        self.color = "red"
        self.space = [[False] * self.GRID_WIDTH for
                      x in range(self.GRID_HEIGHT)]

    # Tick the robot's predicted position and update
    # the internal model with new
    # spots cleaned.
    def dead_reckon(self, acceleration):
        pass

    # Update the internal model based on the internal predicted position
    def update(self):
        print("Updating internal map")

    # only for testing.
    def _update_ground_truth(self, owner, screen, pygame):
        print("Updating internal map")
        [shape] = owner.body.shapes  # In pixels
        print("BB: ", shape.bb.left, shape.bb.right, shape.bb.top, shape.bb.bottom)
        print("Test distance: ", shape.point_query((1000, 450)).distance)

        # Get min and max top corners of grid squares near the circle. In pixels, rounded to
        # a multiple of the pixels that 1 square takes up (RES).
        def reduce_to_multiple(x): return x - (x % (InternalModel.GRID_BOX_PX))

        start_x = max(0, reduce_to_multiple(shape.bb.left))
        start_y = max(0, reduce_to_multiple(shape.bb.bottom))
        end_x = min(PARAMS.SURFACE_DIMS_M[0] * PARAMS.PX_PER_M,
                    reduce_to_multiple(shape.bb.right))
        end_y = min(PARAMS.SURFACE_DIMS_M[1] * PARAMS.PX_PER_M,
                    reduce_to_multiple(shape.bb.top))
        print("Bounds: ")

        # Use arange to iterate floats, checking if these are inside the circle.
        # x in pixels.
        loops = 0
        for x in np.arange(start_x, end_x, InternalModel.GRID_BOX_PX):
            for y in np.arange(start_y, end_y, InternalModel.GRID_BOX_PX):
                grid_center_px = (x + (InternalModel.GRID_BOX_PX / 2), y + (InternalModel.GRID_BOX_PX / 2))
                print("Point: ", x, y, "Dist: ", shape.point_query(grid_center_px).distance)
                loops += 1
                pygame.draw.rect(screen, pygame.Color(255, 0, 0),
                                 pygame.Rect(x,
                                             y,
                                             InternalModel.GRID_BOX_PX,
                                             InternalModel.GRID_BOX_PX))
                if (shape.point_query(grid_center_px).distance <= 0):
                    print("point within circle!")
        print("Loops: ", loops)
        # The point query should be in pixel coordinates.
        # For those that are, set them to true in the space.

    def merge(model):
        print("Merge data models here")

    def visualize(self, screen, pygame):
        print("visualize")
        #pygame.draw.rect(screen, pygame.Color(255, 0, 0),
        #                  pygame.Rect(0, 0, InternalModel.RES * PARAMS.PX_PER_M,
        #                              InternalModel.RES * PARAMS.PX_PER_M))

        for x in range(InternalModel.GRID_WIDTH):
            for y in range(InternalModel.GRID_HEIGHT):
                if (self.space[y][x]):
                    print("Drawing a true square!")
                    pygame.draw.rect(screen, pygame.Color(255, 0, 0),
                                     pygame.Rect(x * InternalModel.RES * PARAMS.PX_PER_M,
                                                 y * InternalModel.RES * PARAMS.PX_PER_M,
                                                 InternalModel.RES * PARAMS.PX_PER_M,
                                                 InternalModel.RES * PARAMS.PX_PER_M))
