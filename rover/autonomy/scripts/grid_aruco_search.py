#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
from std_msgs.msg import Float32
import numpy as np

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    print("X: ", x)
    print("Y: ", y)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print("ANGLES: ", (roll, pitch, theta))
    rospy.init_node("speed_controller")

sub = rospy.Subscriber("/stereo_odometry", Odometry, newOdom) # launch zed camera
pub = rospy.Publisher("drive", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

path_list = [(0,0), (3.5,0.0), (3.5, 3.5), (-3.5,3.5), (-3.5, -7.0), (10.5, -7.0), (10.5,10.5), (-10.5,10.5), 
            (-10.5, -14.0), (17.5, -14.0), (17.5, 17.5), (-17.5, 17.5), (-17.5, -21.0), (17.5, -21.0)]
point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.
scale_factor = 0.5
goal = Point ()
while not rospy.is_shutdown():
    if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
        goal.x = path_list[point_index][0] * scale_factor # x coordinate for goal
        goal.y = path_list[point_index][1] * scale_factor # y coordinate for goal
    else:
        break # I guess we're done?

    inc_x = goal.x - (x*scale_factor)
    inc_y = goal.y - (y*scale_factor) 

    angle_to_goal = math.atan2 (inc_y, inc_x) # this is our "bearing to goal" as I can guess

    # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula
    point_distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)

    if point_distance_to_goal >= 0.5: # we'll now head to our target
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3   
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)

    else:
        point_index += 1 

    r.sleep()
    
    
    