#!/usr/bin/env python3


"""
Filename: listener-truprecision-rsx.py

Listen to TruPrecision serial data sent from Windows computer on local network over socket

Server-side (Ubuntu, ROS) script

Robotics for Space Exploration - University of Toronto

Author: Jason Li <jasonli.li@mail.utoronto.ca>
Date: 2024-11-16
"""





import socket
import rospy
from std_msgs.msg import UInt8





# Ensure that these are the same on the Windows-side Python script.
# HOST should be the Ubuntu computer's local IP, and
# PORT should be an available port on the Ubuntu computer.
# This is to send the data to the Ubuntu computer.
HOST = "192.168.0.32"
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

    print("Ready for connection from Windows")


    while not rospy.is_shutdown():

    
        while True:

            # Found connection
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")


                while True:

                    # Received data
                    data = conn.recv(1024)
                    if data:

                        for byte in data:
                            pub.publish(byte)

                    # Disconnect when data has stopped
                    # Outer while loop will wait for connection again
                    else:
                        break

    rospy.loginfo("Shutting down TruPrecision ROS listener node.")


# -----------------------------------------------
