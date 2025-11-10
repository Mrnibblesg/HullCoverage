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

    def tick(self):
        self.planner()  # Set target parameters and control signals
        self.communicate()
        self.move()  # Apply forces from motors

    def planner(self):
        print('Update coverage map')
        self.internal_model.dead_reckon(self.IMU.acceleration)
        #self.internal_model.update()
        self.internal_model._update_ground_truth(self)
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
        self.rotation /= -3

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

        self.acceleration = {
            "psi": 0,
            "forward": 0,
            "lateral": 0
        }

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

        self.acceleration["psi"] = accel_psi
        self.acceleration["forward"] = accel_forward
        self.acceleration["lateral"] = accel_lateral

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
        self.prediction = {
            "psi": 0,
            "x": position[0],
            "y": position[1],

            "v_psi": 0,
            "vx": 0,
            "vy": 0
        }
        self.last_measurement = -1
        self.space = [[False] * self.GRID_WIDTH for
                      x in range(self.GRID_HEIGHT)]

        self.color = "red"

    # Tick the robot's predicted position and update
    # the internal model with new
    # spots cleaned using trapezoid rule.
    # Assuming the acceleration is instantaneous is probably a source of drift
    def dead_reckon(self, acceleration):
        print(acceleration)
        d_time = Robot.world.simulation_time - self.last_measurement

        dx = 0
        dy = 0
        d_psi = 0

        d_vx = 0
        d_vy = 0
        d_v_psi = 0

        d_v_psi = d_time * acceleration["psi"]
        self.prediction["v_psi"] += d_v_psi
        d_psi = d_time * self.prediction["v_psi"] - (d_time * d_v_psi)
        self.prediction["psi"] += d_psi


        #self.prediction.x += dx
        #self.prediction.y += dy

        #self.prediction.v_psi += d_v_psi
        #self.preditction.vx += d_vx
        #self.prediction.vy += d_vy
        self.last_measurement = Robot.world.simulation_time

    # Update the internal model based on the internal predicted position
    def update(self):
        print("Updating internal map")
        self.update_grid((self.prediction.x, self.prediction.y))

    # only for testing. Still use position, but use the predicted position instead of ground-truth.
    def _update_ground_truth(self, owner):
        self.update_grid(owner.body.position)

    def update_grid(self, position):
        print("Updating internal map")
        # The source of our ground truth

        adjusted_rad = Robot.radius * PARAMS.PX_PER_M
        left = position.x - adjusted_rad
        right = position.x + adjusted_rad
        top = position.y + adjusted_rad
        bottom = position.y - adjusted_rad

        def robot_contains(point):
            dx = position[0] - point[0]
            dy = position[1] - point[1]
            return (dx * dx) + (dy * dy) <= adjusted_rad * adjusted_rad

        # Get min and max top corners of grid squares near the circle. In pixels, rounded to
        # a multiple of the pixel size of 1 grid square.
        def reduce_to_multiple(x): return x - (x % (InternalModel.GRID_BOX_PX))

        start_x = max(0, reduce_to_multiple(left))
        start_y = max(0, reduce_to_multiple(bottom))
        end_x = min(PARAMS.SURFACE_DIMS_M[0] * PARAMS.PX_PER_M,
                    reduce_to_multiple(right))
        end_y = min(PARAMS.SURFACE_DIMS_M[1] * PARAMS.PX_PER_M,
                    reduce_to_multiple(top))

        # Use arange to iterate floats, checking if these are inside the circle.
        # x in pixels.
        for x in np.arange(start_x, end_x + InternalModel.GRID_BOX_PX, InternalModel.GRID_BOX_PX):
            for y in np.arange(start_y, end_y + InternalModel.GRID_BOX_PX, InternalModel.GRID_BOX_PX):
                grid_center_px = (x + (InternalModel.GRID_BOX_PX / 2), y + (InternalModel.GRID_BOX_PX / 2))
                # The point query should be in pixel coordinates.
                if (robot_contains(grid_center_px)):
                    # Convert x and y to grid indices to set to true
                    xi = math.floor(x / InternalModel.GRID_BOX_PX)
                    yi = math.floor(y / InternalModel.GRID_BOX_PX)
                    self.space[yi][xi] = True

    def merge(model):
        print("Merge data models here")

    def visualize(self, screen, pygame):
        print("visualize")

        for x in range(InternalModel.GRID_WIDTH):
            for y in range(InternalModel.GRID_HEIGHT):
                if (self.space[y][x]):
                    pygame.draw.rect(screen, pygame.Color(255, 0, 0),
                                     pygame.Rect(x * InternalModel.RES * PARAMS.PX_PER_M,
                                                 y * InternalModel.RES * PARAMS.PX_PER_M,
                                                 InternalModel.RES * PARAMS.PX_PER_M,
                                                 InternalModel.RES * PARAMS.PX_PER_M))
