import pygame
import pymunk
import pymunk.pygame_util
import math
from pymunk.vec2d import Vec2d
from world import World
from params import PARAMS
import robot
import matplotlib
# TODO: Only if headless
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

robots = []

# The real representation of the surface. Squares are True if covered,
# False otherwise.
# Currently the same as the internal representation. Should be changed to
# be more representative of the real world, probably...
# We should really get a handle on how we want
# to represent our environments better.
screen = None
world = World()
robot.Robot.world = world
draw_options = None


def init():
    global screen
    global draw_options
    pygame.init()
    screen = pygame.display.set_mode(PARAMS.WINDOW_DIMS)
    # Our robot is magnetically attached, so we have high friction

    # and don't need to account for slipping.
    world.space.damping = 0.01

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    r = robot.Robot(Vec2d(PARAMS.SURFACE_DIMS_M[0] * PARAMS.PX_PER_M / 2,
                          PARAMS.SURFACE_DIMS_M[1] * PARAMS.PX_PER_M / 2))
    robots.append(r)
    world.space.add(r.body, r.shape)

    # TODO only when !headless & debug

    loop()


def loop():
    running = True
    step_interval = 1 / PARAMS.FRAME_RATE
    # Only if HEADLESS
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for i, r in enumerate(robots):
            r.tick()

        world.space.step(step_interval)
        pygame.display.flip()
        world.clock.tick(PARAMS.FRAME_RATE)
        world.simulation_time += step_interval

        if world.simulation_time >= PARAMS.RUN_TIME:
            running = False
        if not PARAMS.HEADLESS:
            draw()
    end()


def draw():
    screen.fill((255, 255, 255))
    for r in robots:
        r.visualize(screen, pygame)
    world.space.debug_draw(draw_options)


def end():
    # TODO Record statistics.
    print("The sim is finished.")


if __name__ == "__main__":
    init()
