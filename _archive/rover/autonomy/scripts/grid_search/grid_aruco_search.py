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
init = 0

def newOdom(msg):
    global x, y, theta, init

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    init += 1

    #print("X: ", x)
    #print("Y: ", y)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print("ANGLES: ", (roll, pitch, theta))
rospy.init_node("speed_controller")

sub = rospy.Subscriber("/rtabmap/odom", Odometry, newOdom) # launch zed camera
pub = rospy.Publisher("drive", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

r = rospy.Rate(5)


scale_factor = 0.75
path_list = [(3.5+x,0.0+y), (3.5+x, 3.5+y), (-3.5+x, 3.5+y)
            ,(-3.5+x, -7.0+y), (10.5+x, -7.0+y), (10.5+x,10.5+y), (-10.5+x,10.5+y), 
             (-10.5+x, -14.0+y), (17.5+x, -14.0+y), (17.5+x, 17.5+y), (-17.5+x, 17.5+y), (-17.5+x, -21.0+y), (17.5+x, -21.0+y)]
point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.

goal = Point ()
while not rospy.is_shutdown():
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
        goal.x = path_list[point_index][0]  # x coordinate for goal
        goal.y = path_list[point_index][1]  # y coordinate for goal
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        break # I guess we're done?

    inc_x = (goal.x - x)*scale_factor
    inc_y = (goal.y - y)*scale_factor

    init_angle_to_goal = math.atan2(inc_y, inc_x) # this is our "bearing to goal" as I can guess
    angle_to_goal = math.atan2(inc_y, inc_x)

    # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula
    point_distance_to_goal = np.sqrt((inc_x*inc_x + inc_y*inc_y))

    #old = 100
    print (goal.x-x)
    print (goal.y-y)
    print ("angle to goal", init_angle_to_goal)
    print ("dist to goal", point_distance_to_goal)
    

    if point_distance_to_goal >= 0.5:
        print (goal.x-x)
        print (goal.y-y)
        print ("angle to goal", init_angle_to_goal)
        print ("dist to goal", point_distance_to_goal)
        if abs(init_angle_to_goal - theta) > 0.2:
             speed.angular.z = 0.2
             speed.linear.x = 0.0
             pub.publish(speed)
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0
            pub.publish(speed)
    else:
        point_index += 1

    # while point_distance_to_goal >= 0.5: # we'll now head to our target
    #     print("x", inc_x)
    #     print("y", inc_y)
    #     print("init_angle_to_goal", init_angle_to_goal)
    #     print("angle_to_goal", angle_to_goal)
    #     print("theta", theta)
    #     print("diff", init_angle_to_goal - theta)
    #     print("point_distance_to_goal", point_distance_to_goal)
    #     if init_angle_to_goal - theta > 0.2 and np.abs(old - theta) < 0.3:
    #         speed.linear.x = 0.0
    #         speed.angular.z = 0.2
    #     # to avoid overshoot + 360 turn overcorrection
    #     elif init_angle_to_goal - theta < -0.2 and np.abs(old - theta) < 0.3:
    #         speed.linear.x = 0.0
    #         speed.angular.z = -0.2
    #     else:
    #         speed.linear.x = 0.3
    #         speed.angular.z = 0.0
    #     pub.publish(speed)
    #     inc_x = (goal.x - x)*scale_factor
    #     inc_y = (goal.y - y)*scale_factor
    #     old = theta
    #     angle_to_goal = math.atan2(inc_y, inc_x)
    #     # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula
    #     point_distance_to_goal = np.sqrt((inc_x*inc_x + inc_y*inc_y))
    #     if inc_x<=0.0 or inc_y<=0.0:
    #         point_index+=1
        
    # else:
    #     point_index += 1 

    r.sleep()
