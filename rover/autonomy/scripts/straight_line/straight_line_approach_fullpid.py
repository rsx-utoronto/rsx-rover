#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import time
rate_hz = 50
rate_duration = 1.0 / rate_hz

def ToEulerAngles(w, x, y, z):
    angles = [0, 0, 0] # [roll, pitch, yaw]

    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    angles[2] = math.atan2(siny_cosp, cosy_cosp)

    return angles

def odom_callback(msg):
    global x, y, heading
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    heading = ToEulerAngles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
    print("XXX",x,y)
    # heading is in radians

def target_callback(msg):
    global target_x, target_y
    target_x = msg.data[0]
    target_y = msg.data[1]

def straight_line_approach(lin_vel, ang_vel):
    
    rclpy.init()
    node = rclpy.create_node('straight_line_approach')

    pub = node.create_publisher(Twist, 'drive', 10)
    node.create_subscription(Odometry, '/rtabmap/odom', odom_callback, 10)
    node.create_subscription(Float64MultiArray, 'target', target_callback, 10)
   
    # rate = node.create_rate(50)
    
    threshold = 0.1 # this threshold is for distance and angle 
    kp = 0.5
    kd = 0.5
    ki = 0.5
    acc_err = 0 # start at 0 and accumulates error.
    err_threshold = 50 # acc_error resets to 0 when it gets to 50
    angle_diff = 0 # because prev_angle_diff needs a value to start

    
    while rclpy.ok():

        msg = Twist()
        if target_x == None or target_y == None or x == None or y == None:
            continue      
        target_heading = math.atan2(target_y - y, target_x - x) # in radians #1. Switch places, 
        print("target heading", target_heading)  
        target_distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        time.sleep(rate_duration)
        print("target_distance", target_distance)
        print(x,y)
        
        prev_angle_diff = angle_diff # storing old angle difference
        angle_diff = target_heading - heading
        print("Angle dist:", angle_diff)
        
        if angle_diff > math.pi: # this makes sure angle is in [-pi, pi] 
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
    
        acc_err += abs(angle_diff) # accumulative error
        if acc_err > err_threshold: # resets to 0 at specifief threshold
            acc_err = 0
        diff_err = abs(prev_angle_diff - angle_diff) # old - new angle difference

        if target_distance < threshold:
            msg.linear.x=float(0)
            msg.angular.z=float(0)
            print("stoping")
            pub.publish(msg)
            # next = True
            break


        if abs(angle_diff) <= threshold: #changed this ti < and =
            delta=target_x-x
            print("target_reached_x",delta)
            print("current_X", x)
            #if delta is negative, go to negative velocity.
            msg.linear.x = float(lin_vel)
            msg.angular.z = 0

        else:
            msg.linear.x = 0
            msg.angular.z = (kp * angle_diff) # + (kd * diff_err) + (ki * acc_err)
            if abs((kp * angle_diff)  + (kd * diff_err) + (ki * acc_err)) < 0.3:
                if angle_diff*kp > 0:
                    msg.angular.z = 0.3    
                else:
                    msg.angular.z = -0.3
                    

            print("angular vel ", msg.angular.z)

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    x = 0
    y = 0
    heading = 0
    target_x = 7
    target_y = -3
    
    
    try:
        straight_line_approach(1.5, 0.5)  # your function
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()