import pygame, pymunk
import pymunk.pygame_util

import robot

pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()

space = pymunk.Space()
space.gravity = (0, 900)
draw_options = pymunk.pygame_util.DrawOptions(screen)

# create a ball
r = robot.Robot()
space.add(r.body, r.shape)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))
    space.debug_draw(draw_options)
    space.step(1/60)
    pygame.display.flip()
    clock.tick(60)

