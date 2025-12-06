# NOTE DID NOT CONVERT TO ROS2 AS WE DO NOT USE THIS FILE

import rospy
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
from math import * 


def gps_to_cartesian(start_lat, start_lon, target_lat, target_lon):
    # Use geopy to calculate the distance and bearing
    start_coords = (start_lat, start_lon)
    target_coords = (target_lat, target_lon)
    
    # Calculate the difference in meters using geodesic
    distance = geodesic(start_coords, target_coords).meters
    
    # Calculate the bearing from start to target
    delta_lat = target_lat - start_lat
    delta_lon = target_lon - start_lon

    # Approximate conversion to Cartesian coordinates
    x = delta_lon * 111320 * cos(start_lat * (pi / 180))
    y = delta_lat * 110540

    return x, y

# start_lat = 38.42390688
# start_lon = -110.7852678
start_lat = 38.42390688
start_lon = -110.7852678

# Target GPS coordinates
target_lat = 38.42426632 # Example target latitude
target_lon = -110.7851849  # Example target longitude

x, y = gps_to_cartesian(start_lat, start_lon, target_lat, target_lon)

# finally rotate this by the current heading of the rover
heading = 10  # Example heading angle from the north direction
y_rotated = x * cos(radians(heading)) - y * sin(radians(heading))
x_rotated = x * sin(radians(heading)) + y * cos(radians(heading))

print(x_rotated, y_rotated)

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
    # heading is in radians

def target_callback(msg):
    global target_x, target_y
    target_x = msg.data[0]
    target_y = msg.data[1]
    

def straight_line_approach(lin_vel, ang_vel):
    rospy.Subscriber('/rtabmap/odom', Odometry, callback=odom_callback) # change topic name
    target_x = x_rotated
    target_y = y_rotated
    # rospy.Subscriber('target', Float64MultiArray, callback=target_callback) # change topic name
    # float64[2] data format: [x, y]
    pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
    rospy.init_node('straight_line_approach')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Twist()
        if target_x == None or target_y == None or x == None or y == None:
            continue
        target_heading = math.atan2(target_y - y, target_x - x) # in radians
        print("target_heading,", target_heading)
        target_distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        print("target distance", target_distance)
        angle_diff = target_heading - heading
        # angle_diff = target_heading
        print("angel diff", angle_diff)
            
            
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
        
        # if distance is close enough stop the rover
        if target_distance < 1:
            msg.linear.x = 0
            msg.angular.z = 0
            # stop all processes
            pub.publish(msg)
            print("preshut")
            rospy.signal_shutdown("Target reached")
            print("shouldn't see me")
            # break
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    x = None
    y = None
    heading = None
    target_x = None
    target_y = None
    try:
        straight_line_approach(0.8, 0.25) # change linear and angular velocities
    except rospy.ROSInterruptException:
        pass
