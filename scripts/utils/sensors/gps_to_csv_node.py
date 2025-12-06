#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv

def callback(data):
    rospy.loginfo("Longitude: %f, Latitude %f, Altitude %f" % (data.longitude, data.latitude, data.altitude))

    f = open('gps.csv', 'a')

    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow([data.longitude, data.latitude, data.altitude])
    f.close()
    
def start():
    rclpy.init()
    node=rclpy.create_node('gps_to_csv_node')
    # rospy.init_node('gps_to_csv', anonymous=True)
    node.create_subscription(NavSatFix, "/fix", callback, 10)
    rclpy.spin(node)

if __name__ == '__main__':
    start()





