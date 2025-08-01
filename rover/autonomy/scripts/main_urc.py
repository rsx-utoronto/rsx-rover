#!/usr/bin/env python3

##NOT IN USE
#note: positive angluar velocity is counter clockwise 
"""
The main file for integrating the different modules of the rover autonomy system for URC 2024, 
will be replaced by State Machine in 2025
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math
import numpy as np
from geometry_msgs.msg import Twist
import os
import aruco_homing
from aruco_homing import Aimer, PID, AimerROS
import ar_detection_node 
from ar_detection_node import ARucoTagDetectionNode
import grid_search 
from grid_search import grid_search_class
import thomas_grid_search 
from thomas_grid_search import thomasgrid
import write_serial

class maincaller(Node):
    def __init__(self, name, gps_type):
        super().__init__('maincaller_node')
        self.name = name
        self.gps_type = gps_type # array
        self.scale_factor = 0.75 # to scale up/down the path
        
    def next(self):
        if self.name == "aruco":
            self.aruco()
        elif self.name == "gps":
            self.gps()
        elif self.name == "object":
            self.object()        
    
    def aruco (self):
       # path_list = [(0,0), (3.5,0.0), (3.5, 3.5), (-3.5, 3.5)]
        #             # ,(-3.5, -7.0), (10.5, -7.0), (10.5,10.5), (-10.5,10.5), 
        #             # (-10.5, -14.0), (17.5, -14.0), (17.5, 17.5), (-17.5, 17.5), (-17.5, -21.0), (17.5, -21.0)]
        
        # jack's detection begins 
        #if nothing is initially detected, start grid search
        # if something is detected, shit from jack's pass
        # if nothing is dete
        # cted, stop everything and give up on task.
        port = '/dev/ttyACM0' # change based on find_usb.sh
        led = write_serial.write_serial(port)
        led.write('a')

        grid =thomasgrid()  
        tag_detector = ARucoTagDetectionNode()
        # if tag_detector.is_found() == False:      
        grid.move(tag_detector)
        
        if grid.go_to_loc == True: 
            print ("MAIN: something is found")  
            at_aruco = aruco_homing.main()
            print("MAIN: at aruco", at_aruco)
            if at_aruco:
                led.write('g')
        else:
            print("MAIN: nothing found")
    
    def gps(self):
        pass
    
    def object (self):
        pass        
        
    
if __name__ == "__main__":
    rclpy.init_node(args=None)
    
    ar=maincaller("aruco", "gps")
    ar.next()

    ar.destroy_node()
    rclpy.shutdown()