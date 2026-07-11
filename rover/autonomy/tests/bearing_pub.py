#!/usr/bin/env python3

import pyzed.sl as sl 
import cv2
import numpy as np
import math
import time 
import rospy
from sensor_msgs.msg import NavSatFix
 
##
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class Angles:
    def __init__(self):
        self.curr_gps = "/ublox/fix"
        self.sensor = sl.SensorsData()
        
    # def angle_get():
    #     sensors_data = sl.SensorsData()
    #     # if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
    #     #     # Check if the data has been updated since the last time            
    #     #     # Check if Magnetometer data has been updated (not the same frequency than IMU)
    #     #     if ts_handler.is_new(sensors_data.get_magnetometer_data()):
    #     #         
    #     magnetic_field_calibrated = sensors_data.get_magnetometer_data().get_magnetic_field_calibrated()
    #     print(" - Magnetometer\n \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], magnetic_field_calibrated[1], magnetic_field_calibrated[2]))
    #     reading = sensors_data.get_magnetometer_data().magnetic_heading
    #     print ((float(reading) - 10.01667 + 360)%360)

    def main(self):
        # Used to store the sensors timestamp to know if the sensors_data is a new one or not
        # ts_handler = TimestampHandler()
        # sensors_data = sl.SensorsData()
        # print(sensors_data)
        print("in main")
        while True:
            # if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
            #     # Check if the data has been updated since the last time            
            #     # Check if Magnetometer data has been updated (not the same frequency than IMU)
            #     if ts_handler.is_new(sensors_data.get_magnetometer_data()):
            #         
            magnetic_field_calibrated = self.sensor.get_magnetometer_data().get_magnetic_field_calibrated()
            print(" - Magnetometer\n \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], magnetic_field_calibrated[1], magnetic_field_calibrated[2]))
            reading = sensors_data.get_magnetometer_data().magnetic_heading
            heading =  ((float(reading) - 10.01667 + 360)%360)
            return heading
        return -1
