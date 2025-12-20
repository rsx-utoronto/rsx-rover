#!/usr/bin/python3
# NOTE DID NOT CONVERT TO ROS2 AS WE DO NOT USE THIS FILE
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
    global target_x, target_y
    target_x = msg.data[0]
    target_y = msg.data[1]

def straight_line_approach(lin_vel, ang_vel):
    rospy.Subscriber('/rtabmap/odom', Odometry, callback=odom_callback) # change topic name
    rospy.Subscriber('target', Float64MultiArray, callback=target_callback) # change topic name
    # float64[2] data format: [x, y]
    pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
    rospy.init_node('straight_line_approach')
    rate = rospy.Rate(10)
    print("till here works")
    while not rospy.is_shutdown():

        msg = Twist()
        if target_x == None or target_y == None or x == None or y == None:
            continue      
        target_heading = math.atan2(target_y - y, target_x - x) # in radians #1. Switch places, 
        print("target heading", target_heading)  
        target_distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        


        print("target_distance", target_distance)
        print(x,y)
        angle_diff = target_heading - heading
        print("Angle dist:", angle_diff)
        if angle_diff > math.pi: #this makes sure angle is in [-pi, pi] 
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if target_distance <0.05:
            msg.linear.x=0
            msg.angular.z=0
            print("stoping")
            pub.publish(msg)
            # next = True
            break


        if abs(angle_diff) <= 0.2: #changed this ti < and =
            delta=target_x-x
            print("target_reached_x",delta)
            print("current_X", x)
            #if delta is negative, go to negative velocity.
            msg.linear.x =lin_vel
            
          
            #can delete this part and just do msg.linear.x =0 
            # if delta>0.5:
            #     print("target still greater than 0 so moving forward")
            #     msg.linear.x = lin_vel
            # elif delta<0.5:
            #     msg.linear.x = -lin_vel
            #     print("going in negative velocity now", -lin_vel)
            # else: 
            #     print("target is iwthin 0.5") 
            #     msg.linear.x = 0
        
            msg.angular.z = 0
            
        elif angle_diff > 0.2: #changes this from 0 to 0.3
            msg.linear.x = 0
            print("this is 1")
            msg.angular.z = ang_vel
        else:
            msg.linear.x = 0
            print('this is 2')
            msg.angular.z = -ang_vel
            
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    x = 0
    y = 0
    heading = 0
    target_x = 7
    target_y = 3
    try:
        straight_line_approach(1.5, 0.5) # change linear and angular velocities
    except rospy.ROSInterruptException:
        pass  