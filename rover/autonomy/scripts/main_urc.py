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
from aruco_homing import Aimer
import ar_detection_node 
from ar_detection_node import ARucoTagDetectionNode
import grid_search 
from grid_search import grid_search_class

class maincaller(object):
    def __init__(self, name, gps_type):
        self.name = name
        self.gps_type = gps_type # array
        self.scale_factor = 0.75
        # self.sub_ar_detection_node = ar_detection_node.aruco_pub
        # self.sub_ar_homing = aruco_homing.
        # self.grid_aruco_search  =
        
    def next(self):
        if self.name == "aruco":
            self.aruco()
        elif self.name == "gps":
            self.gps()
        elif self.name == "object":
            self.object()        
    
    def aruco (self):
        path_list = [(0,0), (3.5,0.0), (3.5, 3.5), (-3.5, 3.5)]
        #             # ,(-3.5, -7.0), (10.5, -7.0), (10.5,10.5), (-10.5,10.5), 
        #             # (-10.5, -14.0), (17.5, -14.0), (17.5, 17.5), (-17.5, 17.5), (-17.5, -21.0), (17.5, -21.0)]
        
        # jack's detection begins 
        #if nothing is initially detected, start grid search
        # if something is detected, shit from jack's pass
        # if nothing is detected, stop everything and give up on task.
        
        tag_detector = ARucoTagDetectionNode()
        grid_s=grid_search_class()  
        if tag_detector.is_found() == False:      
            grid_s.follow_path(self, path_list, self.scale_factor, tag_detector)
        if grid_s.is_go_to_loc == True: 
            
            
        else:
            print("nothign found")
            
        
            
       
            
            


        
    
    def gps(self):
        pass
    
    def object (self):
        pass        
        
    
if __name__ == "__main__":
    ar=maincaller(aruco, gps..)
    

    