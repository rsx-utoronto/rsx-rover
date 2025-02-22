#!/usr/bin/env python3


"""
Filename: listen-publish-truprecision.py

Listen to TruPrecision serial data sent from Windows computer on local network over socket and publish ROS topic

Server-side (Ubuntu, ROS) script

Client-side code script is available at
https://github.com/rsx-utoronto/calian-windows-transfer/blob/main/reader-truprecision.py

Robotics for Space Exploration - University of Toronto

Author: Jason Li <jasonli.li@mail.utoronto.ca>
Date: 2024-11-23
"""





import socket
import rospy
from std_msgs.msg import UInt8





# Ensure that these are the same on the Windows-side Python script.
# HOST should be the Ubuntu computer's local IP, and
# PORT should be an available port on the Ubuntu computer.
# This is to send the data to the Ubuntu computer.
HOST = "192.168.2.88"
PORT = 5409





# ROS setup -------------------------------------


rospy.init_node('listener_truprecision', anonymous=True)
topic_name = 'truprecision_data'
pub = rospy.Publisher(topic_name, UInt8, queue_size=10)


# -----------------------------------------------





# Socket listener -------------------------------


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()

    rospy.loginfo("Ready for connection from Windows")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Found connection
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")


            while not rospy.is_shutdown():

                # Received data
                data = conn.recv(1024)
                if data:

                    for byte in data:
                        pub.publish(byte)

                # Disconnect when data has stopped
                # Outer while loop will wait for connection again
                else:
                    break
                rate.sleep()

        rospy.loginfo("Shutting down TruPrecision ROS listener node.")
        rate.sleep()


# -----------------------------------------------
