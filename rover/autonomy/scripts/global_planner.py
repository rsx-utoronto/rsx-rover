#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
from std_msgs.msg import Float32

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

goal = Point()
goal.x = 3
goal.y = 0.5

pos_integral = 0
Kpp = 0.1
Kdp = 0.05
Kip = 0.001
previous_pos_error = 0

angle_integral = 0
Kpa = 0.1
Kda = 0.05
Kia = 0.001
previous_angle_error = 0


while not rospy.is_shutdown():
    inc_x = goal.x -x
    print("inc x:", inc_x)
    inc_y = goal.y -y
    print("inc y:", inc_y)

    pos_error = math.sqrt(inc_x**2 + inc_y**2)

    angle_to_goal = math.atan2(inc_y, inc_x)
    print("angle to goal: ", angle_to_goal)

    angle_error = angle_to_goal - theta


    P = 0.1

    print("pos_error: ", pos_error)
    print("angle_error: ", angle_error)

    angle_integral += angle_error
    angle_derivative = ((angle_error)- (previous_angle_error))
    if angle_integral > 5:
        angle_integral = 5
    angle_correction = (Kpa*angle_error) + (Kia*angle_integral) + (Kda*angle_derivative)
    pos_integral += pos_error
    pos_derivative = ((pos_error)- (previous_pos_error))
    if pos_integral > 5:
        pos_integral = 5
    pos_correction = (Kpp*pos_error) + (Kip*pos_integral) + (Kdp*pos_derivative)

    previous_pos_error = pos_error
    previous_angle_error = angle_error

    if abs(angle_error) < 0.3:
        if pos_error <= 0.5:
            speed.linear.x = 0.0
            pub.publish(speed)
            if abs(angle_error) < 0.2: 
                speed.angular.z = 0.0
                pub.publish(speed)
            else: 
                # if angle_correction > 0:
                speed.angular.z = min(0.5, angle_correction)
                # else:
                #     speed.angular.z = max(-0.5, angle_correction)
                pub.publish(speed)


        else:
            #if you have gone past: 
            #1. make speed negative and oscillate 
            #2. don't do pos_correction 
            speed.angular.z = 0.0
            speed.linear.x = min(0.3, P*(0.02 + pos_correction))
            pub.publish(speed)
    else:
        speed.angular.z = min(0.5, angle_correction)

        speed.linear.x = 0.0
        pub.publish(speed)

    
    
    print("Speed: ", speed)
    pub.publish(speed)
    pub_error.publish(pos_error)
    r.sleep() 

