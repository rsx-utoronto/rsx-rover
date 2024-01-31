#! /usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge 

# create the name of publisher node 
publisher_name = "camera_sensor_publisher"

# create the name of the topic over whcih will transmit the iamge messages 
topicName = "video_topic"

# initialize the node 
rospy.init_node(publisher_name, anonymous=True)

# create a publisher object, specify the name of the topic, a tpye of message beign sent (Image), 
# and define the buffer size (queue_size)
publisher = rospy.Publisher(topicName, Image, queue_size=60)

# rate of transmitting the messages 
rate = rospy.Rate(60)

video = cv2.VideoCapture(0)

# create the CvBridge object that will be used to convert OpenCV Images to ROS image messages
bridgeObject = CvBridge()

# captures the iamges and transmits them through the topic
while not rospy.is_shutdown():
    returnValue, capturesFrame = video.read()
    if returnValue:
        rospy.loginfo('Video Frame captured and published')
        # convert OpenCV to ROS image message
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturesFrame)
        # pubish the converted image throught the topic
        publisher.publish(imageToTransmit)

    rate.sleep()