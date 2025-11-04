import pygame
import pymunk


class World:
    clock = None
    space = None

    def __init__(self):
        self.space = pymunk.Space()
        self.clock = pygame.time.Clock()
        self.simulation_time = 0
