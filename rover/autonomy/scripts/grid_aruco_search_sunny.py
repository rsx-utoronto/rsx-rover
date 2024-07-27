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

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
rospy.init_node("speed_controller")

sub = rospy.Subscriber("/rtabmap/odom", Odometry, newOdom) # launch zed camera
pub = rospy.Publisher("drive", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)



path_list = [(0+x,0+y), (3.5+x,0.0+y), (3.5+x, 3.5+y), (-3.5+x, 3.5+y)
            ,(-3.5, -7.0), (10.5, -7.0), (10.5,10.5), (-10.5,10.5), 
            (-10.5, -14.0), (17.5, -14.0), (17.5, 17.5), (-17.5, 17.5), (-17.5, -21.0), (17.5, -21.0)]
point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.
scale_factor = 0.75
goal = Point ()
index = 0
init_ang = theta
prev_point_index = -1
prev_distance = 1000000
state = 0  #0  moving, 1 turning
while not rospy.is_shutdown():
    if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
        goal.x = path_list[point_index][0]  # x coordinate for goal
        goal.y = path_list[point_index][1]  # y coordinate for goal
    else:
        break # I guess we're done?
    print ("yaw", theta)
    goal.x = path_list[index][0]  # x coordinate for goal
    goal.y = path_list[index][1]  # y coordinate fsor goal
    goal_ang = init_ang+math.pi/2*index
    inc_x = (goal.x - x)*scale_factor**index
    inc_y = (goal.y - y)*scale_factor**index
    cur_point_distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
    print(cur_point_distance_to_goal, speed.linear.x)

    if state == 0:
        # cur_point_distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
        d_dist = cur_point_distance_to_goal - prev_distance
        if d_dist > 0.1:
            index = index + 1
            state = 1
        elif cur_point_distance_to_goal > 1:
            speed.linear.x = 1
            speed.angular.z = 0
        
        elif cur_point_distance_to_goal < 1:
            speed.linear.x=0.3
            # while cur_point_distance_to_goal>0.3:
            #     pass
        # print(speed.linear.x, speed.angular.z)

        prev_distance = cur_point_distance_to_goal
    print(d_dist)
    if state == 1:
        print(theta, goal_ang)
        if theta<goal_ang-0.3:
            speed.linear.x= 0
            speed.angular.z = 0.3
        elif theta < goal_ang:
            speed.linear.x = 0
            speed.angular.z = 0.2
        else:
            state = 0
            prev_distance = 1000000
    pub.publish(speed)

    
    # angle_to_goal = math.atan2 (inc_y, inc_x) # this is our "bearing to goal" as I can guess
    # angle_to_goal = theta + math.pi/2

    # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula

    r.sleep()