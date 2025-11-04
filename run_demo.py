import pygame
import pymunk
import pymunk.pygame_util
from pymunk.vec2d import Vec2d
from world import World
from params import PARAMS
import robot

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

    r = robot.Robot(Vec2d(5, 5))
    robots.append(r)
    world.space.add(r.body, r.shape)
    loop()


def loop():
    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if not PARAMS.HEADLESS:
            draw()
        for r in robots:
            r.tick()
        world.space.step(1/PARAMS.FRAME_RATE)
        pygame.display.flip()
        world.clock.tick(PARAMS.FRAME_RATE)
        world.simulation_time += world.clock.get_time()
        if world.simulation_time >= PARAMS.RUN_TIME * 1000:
            running = False
    end()


def draw():
    screen.fill((255, 255, 255))
    line_color = pygame.Color(200, 200, 200)
    pygame.draw.rect(screen, line_color,
                     pygame.Rect(PARAMS.PADDING_PX, PARAMS.PADDING_PX,
                                 PARAMS.WINDOW_DIMS[0] - 2 * PARAMS.PADDING_PX,
                                 PARAMS.WINDOW_DIMS[1] - 2 * PARAMS.PADDING_PX),
                     1)
    world.space.debug_draw(draw_options)


def end():
    # TODO Record statistics.
    print("The sim is finished.")


if __name__ == "__main__":
    init()
