#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

def ToEulerAngles(w, x, y, z):
    angles = [0, 0, 0] # [roll, pitch, yaw]

    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = math.atan2(sinr_cosp, cosr_cosp);

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
    # heading is in radians

def target_callback(msg):
    global target_x, target_y
    target_x = msg.data[0]
    target_y = msg.data[1]

def straight_line_approach(lin_vel, ang_vel):
    rospy.Subscriber('odom', Odometry, callback=odom_callback) # change topic name
    rospy.Subscriber('target', Float64MultiArray, callback=target_callback) # change topic name
    # float64[2] data format: [x, y]
    pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
    rospy.init_node('straight_line_approach')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Twist()
        if target_x == None or target_y == None or x == None or y == None:
            continue
        target_heading = math.atan2(target_y - y, target_x - x) # in radians
        target_distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        angle_diff = target_heading - heading
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        if abs(angle_diff) < 0.1:
            msg.linear.x = lin_vel
            msg.angular.z = 0
        elif angle_diff > 0:
            msg.linear.x = 0
            msg.angular.z = ang_vel
        else:
            msg.linear.x = 0
            msg.angular.z = -ang_vel
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    x = None
    y = None
    heading = None
    target_x = None
    target_y = None
    try:
        straight_line_approach(1, 1) # change linear and angular velocities
    except rospy.ROSInterruptException:
        pass