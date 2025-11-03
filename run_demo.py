import pygame
import pymunk
import pymunk.pygame_util

import robot

HEADLESS = False

# For the current model of the surface, the space is represented as a
# vertical square, where the top of the square is at the surface of the water
# and the bottom is under the surface of the water.
# We assume our robot is 1m in diameter.

RUN_TIME = 15  # Seconds
FRAME_RATE = 60  # FPS

SURFACE_RESOLUTION = 3  # pixels
GRID_DIMS = (250, 250)  # grid squares

PADDING = 25
WINDOW_DIMS = (min(GRID_DIMS[0] * SURFACE_RESOLUTION + 2 * PADDING, 1920),
               min(GRID_DIMS[1] * SURFACE_RESOLUTION + 2 * PADDING, 1080))

robots = []

# The real representation of the surface. Squares are True if covered,
# False otherwise.
# Currently the same as the internal representation. Should be changed to
# be more representative of the real world, probably...
# We should really get a handle on how we want
# to represent our environments better.
grid = [[False] * GRID_DIMS[0]] * GRID_DIMS[1]

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
    screen = pygame.display.set_mode(WINDOW_DIMS)
    clock = pygame.time.Clock()
    space = pymunk.Space()
    space.damping = 0.1

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    r = robot.Robot((350,50))
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
        if not HEADLESS:
            draw()
        for r in robots:
            r.tick()
        space.step(1/FRAME_RATE)
        pygame.display.flip()
        clock.tick(FRAME_RATE)
        time = time + clock.get_time()
        if time >= RUN_TIME * 1000:
            running = False
    end()


def draw():
    screen.fill((255, 255, 255))
    line_color = pygame.Color(200, 200, 200)
    pygame.draw.rect(screen, line_color,
                     pygame.Rect(PADDING, PADDING,
                                 SURFACE_RESOLUTION * GRID_DIMS[0],
                                 SURFACE_RESOLUTION * GRID_DIMS[1]),
                     1)
    space.debug_draw(draw_options)


def end():
    # TODO Record statistics.
    print("The sim is finished.")


if __name__ == "__main__":
    HEADLESS = False
    init()
