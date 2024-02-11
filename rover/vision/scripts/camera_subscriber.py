#! /usr/bin/env python 
import rospy 
from PIL import Image as im
from sensor_msgs.msg import Image 
import cv2 
from ultralytics import YOLO
from roboflow import Roboflow 
from cv_bridge import CvBridge 

API_KEY = "B90CKEnxRHNGK992SHdt"

# create the name of publisher node 
subscriberNodeName = "camera_sensor_subscriber"

# make sure that the same name is used in the soufce file of the publisher
topicName = "video_topic"

rf = Roboflow(api_key = API_KEY)
project = rf.workspace().project("obstacle-detecting")
model = project.version('3').model


# function callback that is called everytume the message arrives 
def callback(message):
    bridgeObject = CvBridge()

    rospy.loginfo("received a video message/frame")

    # convert from cv_bridge to OpenCV image format
    convertedFrame = bridgeObject.imgmsg_to_cv2(message)
    # receive the cv2 image
    path_img = "output.png"
    cv2.imwrite(path_img, convertedFrame)

    # frame shape is (480, 640, 3)
    result = model.predict(path_img, confidence = 50, overlap = 30).json()
    
    # rospy.loginfo(str(result))

    predictions = result['predictions']
    for prediction in predictions:
        if prediction['confidence'] > 0.5:
            image = cv2.imread(path_img)
            center_x = int(prediction['x'])
            center_y = int(prediction['y'])
            predict_width = int(prediction['width'])
            predict_height = int(prediction['height'])
            x, y = int(center_x - predict_width/2), int(center_y - predict_height/2)
       
            convertedFrame = cv2.rectangle(image, (x, y), (x+predict_width, y+predict_height), (0, 255, 0), 2)

    string = str(result)
    # rospy.loginfo(string)


    cv2.imshow("camera", convertedFrame)

    cv2.waitKey(1)
    

# initialize the subscriber node 
# anonymous = True means that a random number is added to the subscriber node name
rospy.init_node(subscriberNodeName, anonymous=True)

# specify the topic name, type of the message will receive, and the name of the callback function 
rospy.Subscriber(topicName,Image,callback)

rospy.spin()

cv2.destroyAllWindows()