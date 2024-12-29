#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
from nav_msgs.msg import Odometry

class GridSearch:
    '''
    All values here should be treated as doubles
    w, h - refers to grid dimensions, ie. if comp tells us that it is within radius of 20 then our grid would be square with 40 by 40
    tolerance   - this is the limitation of our own equipment, ie. from camera, aruco only detected left right and dist of 3 m
                - if this is unequal, take the smallest tolerance
    '''
    def __init__(self, w, h, tolerance, lin_vel, ang_vel):
        self.x = w 
        self.y = h
        self.tol = tolerance 

    '''
    Generates a list of targets for straight line approach.
    '''
    def square_target(self):
        x, y, dx, dy = 0, 0, 0, 1
        targets = [(x,y)]
        step = 1

        while len(targets) < self.x:
            for i in range(2):  # 1, 1, 2, 2, 3, 3... etc
                for j in range(step):
                    if step*self.tol > self.x: 
                        break
                    x, y = x + (self.tol*dx), y + (self.tol*dy)
                    targets.append((x, y))
                dx, dy = -dy, dx
            step += 1  
        return targets

        # notice, up/down is odd amount, left/right is even amount        
        # while len(targets) < self.x: # 1, 2, 3, 4, 5... sol 
        #     if step%2==1: # moving up/down
        #         for i in range (step):
        #             # move up if %4==1, down if %4 ==3 from the pattern
        #             y+=1 if step%4 ==1 else -1
        #             targets.append((x,y))
        #             if len(targets)==self.x:
        #                 return targets
        #         else: 
        #             for i in range (step):
        #             # move right if %4==2, left if %4 ==0 
        #                 x+=1 if step%4==2 else -1
        #                 targets.append((x,y))
        #                 if len(targets)==self.x:
        #                     return targets
        #     step += 1  # Increase step size after two edges
        # return targets