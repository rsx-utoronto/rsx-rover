import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
from straight_line_approach import StraightLineApproach
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
        self.straight_line_approach = StraightLineApproach(lin_vel, ang_vel, self.square_target)
        self.odom_subscriber = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]

    '''
    Generates a list of targets for straight line approach.
    '''
    def square_target(self):
        # notice, up/down is odd amount, left/right is even amount        
        
        factor = self.tol
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

    def search(self):
        self.straight_line_approach.navigate()