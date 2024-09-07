#!/usr/bin/python3
#full working version of straight line!

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

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
    global targets
    targets = msg.data

def straight_line_approach(lin_vel, ang_vel):
    rospy.Subscriber('/rtabmap/odom', Odometry, callback=odom_callback) 
    rospy.Subscriber('target', Float64MultiArray, callback=target_callback) 
    pub = rospy.Publisher('drive', Twist, queue_size=10) 
    rospy.init_node('straight_line_approach')
    rate = rospy.Rate(50)
    kp = 0.5

    while not rospy.is_shutdown():
        msg = Twist()
        if targets is None or x is None or y is None:
            rospy.loginfo("Waiting for valid target or position...")
            continue
        
        for target_x, target_y in targets:
            print(f"Moving towards target: ({target_x}, {target_y})")
            
            while not rospy.is_shutdown():
                msg = Twist()
                if target_x is None or target_y is None or x is None or y is None:
                    continue      
                target_heading = math.atan2(target_y - y, target_x - x) # in radians 
                target_distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
                angle_diff = target_heading - heading
                
                if angle_diff > math.pi: # Ensure angle is in [-pi, pi] 
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                threshold = 0.1

                if target_distance < threshold:
                    msg.linear.x = 0
                    msg.angular.z = 0
                    pub.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    break

                if abs(angle_diff) <= threshold:
                    msg.linear.x = lin_vel
                    msg.angular.z = 0
                else:
                    msg.linear.x = 0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                pub.publish(msg)
                rate.sleep()
            
            rospy.sleep(1)  # pause 1 second before moving to next target

if __name__ == '__main__':
    x = 0
    y = 0
    heading = 0
    try:
        straight_line_approach(1.5, 0.5)
    except rospy.ROSInterruptException:
        pass