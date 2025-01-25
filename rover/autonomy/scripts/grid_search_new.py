#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
from std_msgs.msg import Float32
import numpy as np
from std_msgs.msg import Bool

x = 0.0
y = 0.0 
theta = 0.0
found = False

def newOdom(msg):
    global x, y, theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def aruco_callback(msg):
    if msg.data == True:
        print('found here')
        found = True


def dist_to_goal(goal_x, goal_y):
    diff_x = (goal_x - x)*scale_factor
    diff_y = (goal_y - y)*scale_factor
    distance_to_goal = np.sqrt(diff_x**2 + diff_y**2)
    return distance_to_goal


def ang_to_goal(goal_x, goal_y):
    diff_x = (goal_x - x)*scale_factor
    diff_y = (goal_y - y)*scale_factor
    angle_to_goal = np.arctan2(diff_y, diff_x)
    return angle_to_goal


rospy.init_node("grid_search")


sub = rospy.Subscriber("/rtabmap/odom", Odometry, newOdom) # launch zed camera
aruco_sub = rospy.Subscriber('aruco_found', Bool, aruco_callback)
pub = rospy.Publisher("drive", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

path_list = [(3.5+x,0.0+y), (3.5+x, 3.5+y), (-3.5+x, 3.5+y)]
            #,(-3.5, -7.0), (10.5, -7.0), (10.5,10.5), (-10.5,10.5), 
            # (-10.5, -14.0), (17.5, -14.0), (17.5, 17.5), (-17.5, 17.5), (-17.5, -21.0), (17.5, -21.0)]
scale_factor = 1.0

while len(path_list) > 0 and not found:
    goal_x, goal_y = path_list.pop(0)
    if not found:
        print('working')
    else:
        print('bad')

    # angle_to_goal = theta + math.pi/2

    # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula
    distance_to_goal = dist_to_goal(goal_x, goal_y)
    angle_to_goal = ang_to_goal(goal_x, goal_y) # this is our "bearing to goal" as I can guess
    

    print ("angle to goal", angle_to_goal)
    print("theta", theta)
    print ("dist to goal", distance_to_goal)

    while distance_to_goal > 0.5 and not found:
        diff_angle = angle_to_goal - theta
        if abs(diff_angle) > 0.15:
            # the rover is off the angle to next point location
            if diff_angle > 0:
                # rotate CCW?
                speed.linear.x = 0.0
                speed.angular.z = 0.3 
            else:
                # rotate CW?
                speed.linear.x = 0.0
                speed.angular.z = -0.3 
        else:
            # we are on the right heading for the next location but to far away, move forward
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        distance_to_goal = dist_to_goal(goal_x, goal_y)
        angle_to_goal = ang_to_goal(goal_x, goal_y)
        print ("angle to goal", angle_to_goal)
        print("theta", theta)
        print ("dist to goal", distance_to_goal)
if not found:
    print('we are finished, aruco not found')
else:
    print('found')
speed.linear.x = 0.0
speed.angular.z = 0.0
pub.publish(speed)
