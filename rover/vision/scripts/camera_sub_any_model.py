#! /usr/bin/env python3
import rospy 
from PIL import Image as im
from sensor_msgs.msg import Image 
import cv2 
import json
from ultralytics import YOLO
from roboflow import Roboflow 
from cv_bridge import CvBridge 

# create the name of publisher node 
subscriberNodeName = "camera_sensor_subscriber"

# make sure that the same name is used in the soufce file of the publisher
topicName = "video_topic"

model = YOLO('yolov8n.pt')

# function callback that is called everytume the message arrives 
def callback(message):
    bridgeObject = CvBridge()

    rospy.loginfo("received a video message/frame")

    # convert from cv_bridge to OpenCV image format
    convertedFrame = bridgeObject.imgmsg_to_cv2(message)
    # receive the cv2 image
    path_img = "output.png"
    cv2.imwrite(path_img, convertedFrame)

    result = model(path_img, save=False, show=False)
    
    image = cv2.imread(path_img)

    for prediction in result:
        json_object = json.loads(prediction.tojson())
        if len(json_object) != 0:
            for object in json_object:
                convertedFrame = cv2.rectangle(image, 
                                            (int(object['box']['x1']), int(object['box']['y1'])), 
                                            (int(object['box']['x2']), int(object['box']['y2'])), 
                                            (0, 255, 0), 2)

    cv2.imshow("camera", convertedFrame)

    cv2.waitKey(1)
    

# initialize the subscriber node 
# anonymous = True means that a random number is added to the subscriber node name
rospy.init_node(subscriberNodeName, anonymous=True)

# specify the topic name, type of the message will receive, and the name of the callback function 
rospy.Subscriber(topicName,Image,callback)

rospy.spin()

cv2.destroyAllWindows()