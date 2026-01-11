import pygame
import pymunk
import pymunk.pygame_util

from pymunk.vec2d import Vec2d
from world import World
from params import PARAMS
import robot
import matplotlib
# TODO: Only if headless
matplotlib.use("TkAgg")

robots = []

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
    # and don't need to account for much slipping. The real value is 0.01
    # but we're using 0.5 for testing purposes.
    world.space.damping = 0.5

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    r = robot.Robot(Vec2d(PARAMS.SURFACE_DIMS[0] / 4,
                          PARAMS.SURFACE_DIMS[1] / 4))
    robots.append(r)
    world.space.add(r.body, r.shape)

    loop()


def loop():
    running = True
    step_interval = 1 / PARAMS.FRAME_RATE
    physics_interval = step_interval / PARAMS.PHYSICS_STEPS_PER_FRAME

    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for s in range(PARAMS.PHYSICS_STEPS_PER_FRAME):
            for i, r in enumerate(robots):
                r.tick()
            world.space.step(physics_interval)
            world.simulation_time += physics_interval

        pygame.display.flip()
        world.clock.tick(PARAMS.FRAME_RATE)

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
