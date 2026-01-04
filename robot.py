# This file contains the code for each cleaning robot.
import pymunk
from pymunk.vec2d import Vec2d
import math
from params import PARAMS


class Robot:
    radius = 30
    world = None

    def __init__(self, position):
        self.body = pymunk.Body(5, pymunk.moment_for_circle(1, 0, self.radius))

        self.body.position = position
        self.shape = pymunk.Circle(self.body, self.radius)

        self.baro = Barometer(self)
        self.IMU = IMU(self)
        self.motor = Motor(self)
        self.internal_model = InternalModel(self.body.position, self.body.angle)

    # The job of the planner is to decide the next
    # goals of the robot. The ultimate goal is full coverage.
    # To this end, we update our coverage map, see if anyone is nearby,
    # and then optionally update our PID controllers using some sort
    # of path finding algorithm

    def tick(self):
        self.IMU.react()
        self.planner()  # Set target parameters and control signals
        # self.communicate()

        self.move()  # Apply forces from motors

    def planner(self):
        self.internal_model.dead_reckon(self.IMU.acceleration)
        self.internal_model.update()
        # self.internal_model._update_ground_truth(self)

        # Change motor signals
        self.motor.angle_controller(self.internal_model.prediction["psi"], math.pi)
        self.motor.velo_controller(0, 0)

    def communicate(self):
        pass

    def move(self):
        direction = self.body.angle
        force_vec = Vec2d(math.cos(direction), math.sin(direction))

        self.body.apply_force_at_local_point(force_vec * self.motor.forward, (0, 0))

        # TODO Alternatively, set the torque. For wheels this is technically a more accurate
        # representation.
        perp_offset = force_vec.perpendicular_normal()
        self.body.apply_force_at_local_point(force_vec * self.motor.rotation,
                                             perp_offset)
        self.body.apply_force_at_local_point(-force_vec * self.motor.rotation,
                                             -perp_offset)


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
    max_power = 30
    rotation = 0
    forward = 0

    def __init__(self, owner):
        super().__init__(owner)
        self.rotation = 0
        self.forward = 0

    # Not a full PID yet
    # apply the full power to get the values to the goal
    def angle_controller(self, current, goal):
        if (Robot.world.simulation_time < 2):
            self.rotation = 0
        else:
            self.rotation = -300

    def velo_controller(self, current, goal):
        if (Robot.world.simulation_time < 2):
            self.forward = 500
        else:
            self.forward = 0
        pass


# TODO Not used.
class Barometer(Sensor):
    def __init__(self, owner):
        super().__init__(owner)
        self.depth = 0

    def measure(self):
        self.depth = self.owner.body.position[1]


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
    def react(self):
        body = self.owner.body

        velocity = body.velocity_at_world_point(body.position)

        ang_vel = body.angular_velocity
        psi = body.angle
        print("Real y vel: ", velocity[1])

        # Use the rotation matrix to translate world-frame velocity to
        # robot-frame components for forward and lateral velocity.
        forward_vel = (velocity[0] * math.cos(psi)) + \
                      (velocity[1] * math.sin(psi))
        lat_vel = (-velocity[0] * math.sin(psi)) + \
                  (velocity[1] * math.cos(psi))
        if (Robot.world.simulation_time < 2):
            print("Calc y vel: ", lat_vel)
        if self.last_measurement < 0:
            self.last_measurement = Robot.world.simulation_time
            self.last_d_forward = forward_vel
            self.last_d_lateral = lat_vel
            self.last_d_psi = ang_vel
            return

        d_time = Robot.world.simulation_time - self.last_measurement

        accel_forward = None
        accel_lateral = None
        accel_psi = None

        accel_psi = (ang_vel - self.last_d_psi) / d_time
        self.last_d_psi = ang_vel

        accel_forward = (forward_vel - self.last_d_forward) / d_time
        self.last_d_forward = forward_vel

        accel_lateral = (lat_vel - self.last_d_lateral) / d_time
        self.last_d_lateral = lat_vel

        if (Robot.world.simulation_time < 2):
            print("Calc y acc: ", accel_lateral)
        # print("Lateral Acceleration: ", accel_lateral)
        # print("Forward Acceleration: ", accel_forward)
        # print("Radial Acceleration: ", accel_psi)

        self.acceleration["psi"] = accel_psi
        self.acceleration["forward"] = accel_forward
        self.acceleration["lateral"] = accel_lateral

        self.last_measurement = Robot.world.simulation_time


# Contains the internal model, basically the bot knowing where it has been
# based on landmarks. This will be shared with neighboring bots
# whenever they are encountered.
class InternalModel:
    # Defines the size of an internal model square in px.
    RES = 15

    GRID_WIDTH = int(PARAMS.SURFACE_DIMS[0] / RES)
    GRID_HEIGHT = int(PARAMS.SURFACE_DIMS[1] / RES)

    def __init__(self, position, angle):
        # Start out with knowledge. Predict the rest
        self.prediction = {
            "psi": angle,
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

    # Tick the robot's predicted position and update the internal model with new
    # spots cleaned using trapezoid rule.
    def dead_reckon(self, acceleration):

        if self.last_measurement < 0:
            self.last_measurement = Robot.world.simulation_time
            return

        d_time = Robot.world.simulation_time - self.last_measurement

        dx = 0
        dy = 0
        d_psi = 0

        d_vx = 0
        d_vy = 0
        d_v_psi = 0

        # Calculate stuff for psi first
        d_v_psi = d_time * acceleration["psi"]
        self.prediction["v_psi"] += d_v_psi

        d_psi = d_time * self.prediction["v_psi"] - (d_time * d_v_psi / 2)
        self.prediction["psi"] += d_psi
        self.prediction["psi"] %= 2 * math.pi

        psi = self.prediction["psi"]
        psi_sensor_frame = -psi
        world_accel = (acceleration["forward"] * math.cos(psi_sensor_frame) -
                       acceleration["lateral"] * math.sin(psi_sensor_frame),
                       acceleration["forward"] * math.sin(psi_sensor_frame) +
                       acceleration["lateral"] * math.cos(psi_sensor_frame))

        # Integrate twice for psi, x, and y.
        d_vx = d_time * world_accel[0]
        self.prediction["vx"] += d_vx

        dx = d_time * self.prediction["vx"] - (d_time * d_vx / 2)
        self.prediction["x"] += dx

        d_vy = d_time * world_accel[1]
        self.prediction["vy"] += d_vy

        dy = d_time * self.prediction["vy"] - (d_time * d_vy / 2)
        self.prediction["y"] += dy

        self.last_measurement = Robot.world.simulation_time

        # print("Final: ", self.prediction)

    # Update the internal model based on the internal predicted position
    def update(self):
        self.update_grid((self.prediction["x"],
                          self.prediction["y"]))

    # only for testing. Use the ground-truth.
    def _update_ground_truth(self, owner):
        self.update_grid((owner.body.position.x,
                          owner.body.position.y))

    def update_grid(self, position):
        left = position[0] - Robot.radius
        right = position[0] + Robot.radius
        top = position[1] + Robot.radius
        bottom = position[1] - Robot.radius

        def robot_contains(point):
            dx = position[0] - point[0]
            dy = position[1] - point[1]
            return (dx * dx) + (dy * dy) <= Robot.radius * Robot.radius

        # Get min and max top corners of grid squares near the circle.
        def reduce_to_multiple(x): return x - (x % (InternalModel.RES))

        start_x = max(0, int(left / InternalModel.RES))
        start_y = max(0, int(bottom / InternalModel.RES))
        end_x = min(InternalModel.GRID_WIDTH, math.ceil(right / InternalModel.RES))
        end_y = min(InternalModel.GRID_HEIGHT, math.ceil(top / InternalModel.RES))

        for x in range(start_x, end_x):
            for y in range(start_y, end_y):
                grid_center = (x * InternalModel.RES + InternalModel.RES / 2,
                               y * InternalModel.RES + InternalModel.RES / 2)
                if (robot_contains(grid_center)):
                    self.space[y][x] = True

    def merge(model):
        pass

    def visualize(self, screen, pygame):
        for x in range(InternalModel.GRID_WIDTH):
            for y in range(InternalModel.GRID_HEIGHT):
                if (self.space[y][x]):
                    pygame.draw.rect(screen, pygame.Color(255, 0, 0),
                                     pygame.Rect(x * InternalModel.RES,
                                                 y * InternalModel.RES,
                                                 InternalModel.RES,
                                                 InternalModel.RES))

        center = Vec2d(self.prediction["x"], self.prediction["y"])
        front_line_end = Vec2d(Robot.radius * math.cos(self.prediction["psi"]),
                               Robot.radius * math.sin(self.prediction["psi"]))
        pygame.draw.circle(screen, pygame.Color(0, 0, 255),
                           center, Robot.radius)

        pygame.draw.line(screen, pygame.Color(0, 0, 0),
                         center, center + front_line_end)
