## This file contains the code for each cleaning robot.
import pymunk
class Robot:
    def __init__(self):
        self.body = pymunk.Body(5, pymunk.moment_for_circle(1, 0, 30))
        self.body.position = (300,50)
        self.shape = pymunk.Circle(self.body, 30)
        
        self.max_speed = 2
    

