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

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom/ekf/enc_imu", Odometry, newOdom)
pub = rospy.Publisher("/drive", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

goal = Point()
goal.x = 1
goal.y = 2

pos_integral = 0
Kpp = 0.1
Kdp = 0.05
Kip = 0.001
previous_pos_error = 0

angle_integral = 0
Kpa = 0.01
Kda = 0.005
Kia = 0.0001
previous_angle_error = 0


while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    pos_error = math.sqrt(inc_x**2 + inc_y**2)

    angle_to_goal = math.atan2(inc_x, inc_y)

    angle_error = angle_to_goal - theta

    P = 1

    print("pos_error: ", pos_error)
    print("angle_error: ", angle_error)

    angle_integral += angle_error
    angle_derivative = ((angle_error)- (previous_angle_error))
    angle_correction = (Kpa*angle_error) + (Kia*angle_integral) + (Kda*angle_derivative)
    pos_integral += pos_error
    pos_derivative = ((pos_error)- (previous_pos_error))
    pos_correction = (Kpp*pos_error) + (Kip*pos_integral) + (Kdp*pos_derivative)

    previous_pos_error = pos_error
    previous_angle_error = angle_error

    if abs(angle_error) < 0.5:
        if pos_error < 0.4:
            if abs(angle_error) < 0.1: 
                speed.angular.z = 0.0
            else: 
                speed.angular.z = max(0.5, (angle_correction))
            speed.linear.x = 0.0
            
            
        else:
                    #if you have gone past: 
        #1. make speed negative and oscillate 
        #2. don't do pos_correction 

            speed.angular.z = 0.0
            speed.linear.x = max(0.5, P*(0.02 + pos_correction))
    else:

        speed.angular.z = max(0.5, (0.2 + angle_correction))
        speed.linear.x = 0.0
    
    
    print("Speed: ", speed)
    pub.publish(speed)
    pub_error.publish(pos_error)
    r.sleep() 