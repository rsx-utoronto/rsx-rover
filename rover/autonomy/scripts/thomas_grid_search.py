import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
import numpy as np
import time
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class thomasgrid():
    def __init__(self):
        rospy.init_node("speed_controller")
        self.speed=Twist()
        self.pub = rospy.Publisher("drive", Twist, queue_size = 1)
        self.r = rospy.Rate(10)
        self.count = 0
        self.distance_travelled=5
        self.go_to_loc = False
        self.found = rospy.Subscriber("aruco_found", Bool)

    def move (self):
        while not rospy.is_shutdown():
            if self.found()==False:

                time_reference = time.time()
                # MOVING
                while self.found()==False:
                    if time.time() - time_reference < self.distance_travelled and self.found()==False:
                        self.speed.linear.x = 0.8
                        self.pub.publish(self.speed)
                    
                    else:
                        self.speed.linear.x = 0.0
                        self.speed.angular.z = 0.0
                        self.pub.publish(self.speed)
                        break
                time_reference = time.time()
            
            
                # TURNING
                while self.found()==False:
                    if time.time() - time_reference < 4 and self.found()==False:
                        self.speed.angular.z = 0.5
                        self.pub.publish(self.speed)
                    else:
                        self.speed.angular.z = 0.0
                        self.count +=1
                        self.pub.publish(self.speed)
                        if self.count > 17:
                            self.stop()
                        break
                self.distance_travelled += 2

                if self.distance_travelled>35:
                    self.nothing_found_at_end()
                
            else:
                
                self.go_to_loc = True  
                self.stop()
                break  
              
            self.r.sleep()  
            
    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.pub.publish(self.speed)
        return True
        
    def nothing_found_at_end(self):
        print("thomas: nothing found at end")
        return False