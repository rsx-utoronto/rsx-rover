import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
from std_msgs.msg import Float32
import numpy as np
import time


rospy.init_node("speed_controller")
pub = rospy.Publisher("drive", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)



distance_travelled = 5

while not rospy.is_shutdown():
    
    time_reference = time.time()
    # MOVING
    while True:
        if time.time() - time_reference < distance_travelled:
            speed.linear.x = 0.5
            pub.publish(speed)
        
        else:
            speed.linear.x = 0.0
            break

    time_reference = time.time()
    
    # TURNING
    while True:
        if time.time - time_reference < 3:
            speed.linear.z = 0.3
            pub.publish(speed)
            
        else:
            speed.linear.z = 0.0
            break

    distance_travelled += 0.5
    
    
    r.sleep()

