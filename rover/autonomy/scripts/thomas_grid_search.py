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
        self.go_to_loc = False

    def move (self, detector):
        while not rospy.is_shutdown():
            if detector.is_found()==False:

                time_reference = time.time()
                # MOVING
                while True and detector.is_found()==False:
                    if time.time() - time_reference < self.distance_travelled and detector.is_found()==False:
                        self.speed.linear.x = 0.8
                        self.pub.publish(self.speed)
                    
                    else:
                        self.speed.linear.x = 0.0
                        self.speed.angular.z = 0.0
                        self.pub.publish(self.speed)
                        break

                    


                time_reference = time.time()
            
            
                # TURNING
                while True and detector.is_found()==False:
                    if time.time() - time_reference < 4 and detector.is_found()==False:
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
                
            else:
                
                self.go_to_loc = True  
                self.stop()
                break  
              
            self.r.sleep()  
            
            

    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.pub.publish(self.speed)
        print ("thomas: at stop")
        self.go_to_loc = True
        
    def nothing_found_at_end(self):
         print("thomas: nothing found at end")