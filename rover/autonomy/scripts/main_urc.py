#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from rover.msg import StateMsg
import math
from std_msgs.msg import Float32, Bool
import numpy as np
import roslib
roslib.load_manifest('sensor_msgs')
import sys
import time
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty, EmptyResponse
import tf2_ros
import os
import aruco_homing
import ar_detection_node 
import grid_aruco_search 

class maincaller(object):
    def __init__(self, name, gps_type):
        self.name = name
        self.gps_type = gps_type
        self.sub_ar_detection_node= ar_detection_node.
        
        
        # want to have these three args. gps, aruco, object
    
    
        
        
        

if __name__ == "__main__":
    gps=maincaller()

    