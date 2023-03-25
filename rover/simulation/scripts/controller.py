#!/usr/bin/env python3
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

sub = rospy.Subscriber("/robot_base_velocity_controller/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot_base_velocity_controller/cmd_vel", Twist, queue_size = 1)
pub_error = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

goal = Point()
goal.x = 5
goal.y = 3

pos_integral = 0
Kpp = 0.1
Kdp = 0.01
Kip = 0.01
previous_pos_error = 0

angle_integral = 0
Kpa = 0.1
Kda = 0.01
Kia = 0.01
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
    
    if abs(angle_error) < 0.5:
        if pos_error < 0.4:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            
        else: 
            pos_integral += pos_error
            pos_derivative = ((pos_error)- (previous_pos_error))
        #if you have gone past: 
        #1. make speed negative and oscillate 
        #2. don't do pos_correction 
            pos_correction = (Kpp*pos_error) + (Kip*pos_integral) + (Kdp*pos_derivative)

            previous_pos_error = pos_error
            speed.angular.z = 0.0

            speed.linear.x = P*(0.5 + pos_correction)
    else:
        angle_integral += angle_error
        angle_derivative = ((angle_error)- (previous_angle_error))
        angle_correction = (Kpa*angle_error) + (Kia*angle_integral) + (Kda*angle_derivative)

        previous_angle_error = angle_error

        speed.angular.z = (0.2 + angle_correction)
        speed.linear.x = 0.0
    

    """ 
    if (abs(pos_error) < 0.01):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    elif abs(angle_error) > 2.5:
        if angle_error < 0:
            speed.linear.x = 0.1
            speed.angular.z = 0.4
        else: 
            speed.linear.x = 0.1
            speed.angular.z = -0.4
    elif abs(angle_error) > 1.57:
        if angle_error < 0:
            speed.linear.x = 0.8
            speed.angular.z = 0.3
        else: 
            speed.linear.x = 0.8
            speed.angular.z = -0.3
    elif abs(angle_error) > 0.1:
        if angle_error < 0:
            speed.linear.x = pos_error*P
            speed.angular.z = 0.2
        else: 
            speed.linear.x = pos_error*P
            speed.angular.z = -0.2
    elif abs(angle_error) < 0.1 and abs(angle_error) > 0.01:
        if angle_error < 0:
            speed.linear.x = pos_error*P
            speed.angular.z = 0.1
        else: 
            speed.linear.x = pos_error*P
            speed.angular.z = -0.1
    else:
        speed.linear.x = pos_error*P
        speed.angular.z = 0.0 
    """
    
    print("Speed: ", speed)
    pub.publish(speed)
    pub_error.publish(pos_error)
    r.sleep() 