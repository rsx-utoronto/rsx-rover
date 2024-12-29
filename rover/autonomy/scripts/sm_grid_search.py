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
    def __init__(self, w, h, tolerance, start_x, start_y):
        self.x = w 
        self.y = h
        self.tol = tolerance 
        # print(self.tol)
        self.start_x = start_x
        self.start_y = start_y
        # rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback) # change topic name
        

    # def odom_callback(self, msg):
    #     self.start_x = msg.pose.pose.position.x
    #     self.start_y = msg.pose.pose.position.y
    #     print(self.start_x)
    #     print(self.start_y)
    
    '''
    Generates a list of targets for straight line approach.
    '''
    def square_target(self):
        dx, dy = 0, 1
        # while self.start_x == None:
        #     # print("Waiting for odom...")
        #     # rospy.loginfo("Waiting for odom...")
        #     continue
        targets = [(self.start_x,self.start_y)]
        step = 1

        while len(targets) < self.x:
            for i in range(2):  # 1, 1, 2, 2, 3, 3... etc
                for j in range(step):
                    if step*self.tol > self.x: 
                        break
                    self.start_x, self.start_y = self.start_x + (self.tol*dx), self.start_y + (self.tol*dy)
                    targets.append((self.start_x, self.start_y))
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