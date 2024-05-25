import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
from std_msgs.msg import Float32
import numpy as np
import time

class thomasgrid():
    def __init__(self):
        rospy.init_node("speed_controller")
        self.speed=Twist()
        self.pub = rospy.Publisher("drive", Twist, queue_size = 1)
        self.r = rospy.Rate(10)
        self.count = 0
        self.distance_travelled=5
        self.r = rospy.Rate(10)
        self.go_to_loc = False

    def move (self, detector):
        while not rospy.is_shutdown() and not detector.isfound():
            time_reference = time.time()
            # MOVING
            while True:
                if time.time() - time_reference < self.distance_travelled:
                    self.speed.linear.x = 0.5
                    self.pub.publish(self.speed)
                
                else:
                    self.speed.linear.x = 0.0
                    break

            time_reference = time.time()
            
            # TURNING
            while True:
                if time.time - time_reference < 3:
                    self.speed.linear.z = 0.3
                    self.pub.publish(self.speed)
                else:
                    self.speed.linear.z = 0.0
                    self.count +=1
                    if self.count > 17:
                        self.stop()
                    break

            self.distance_travelled += 0.5
            self.r.sleep()

    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.go_to_loc = True
        
    def nothing_found_at_end(self):
         print("nothing found at end")
            

