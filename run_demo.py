import pygame
import pymunk
import pymunk.pygame_util
from pymunk.vec2d import Vec2d
from params import PARAMS
import robot

# For the current model of the surface, the space is represented as a
# vertical square, where the top of the square is at the surface of the water
# and the bottom is under the surface of the water.
# We assume our robot is 1m in diameter.

robots = []

# The real representation of the surface. Squares are True if covered,
# False otherwise.
# Currently the same as the internal representation. Should be changed to
# be more representative of the real world, probably...
# We should really get a handle on how we want
# to represent our environments better.
screen = None
clock = None
space = None
draw_options = None


def init():
    global screen
    global clock
    global space
    global draw_options
    pygame.init()
    screen = pygame.display.set_mode(PARAMS.WINDOW_DIMS)
    clock = pygame.time.Clock()
    space = pymunk.Space()
    space.damping = 0.1

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    r = robot.Robot(Vec2d(5, 5))
    robots.append(r)
    space.add(r.body, r.shape)
    loop()


def loop():
    time = 0
    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if not PARAMS.HEADLESS:
            draw()
        for r in robots:
            r.tick()
        space.step(1/PARAMS.FRAME_RATE)
        pygame.display.flip()
        clock.tick(PARAMS.FRAME_RATE)
        time = time + clock.get_time()
        if time >= PARAMS.RUN_TIME * 1000:
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
    space.debug_draw(draw_options)


def end():
    # TODO Record statistics.
    print("The sim is finished.")


if __name__ == "__main__":
    init()
