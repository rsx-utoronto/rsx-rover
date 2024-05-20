#! /usr/bin/env python3

import os
import rospy 
from PIL import Image as im
from sensor_msgs.msg import Image 
import cv2 
import json
from ultralytics import YOLO 
from cv_bridge import CvBridge 
import cvzone
import math
import numpy as np

# VIDEOS_DIR = os.path.join('.', 'videos')

# video_path = os.path.join(VIDEOS_DIR, 'test1.mp4')
# video_path_out = '{}_out.mp4'.format(video_path)

# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()
# H, W, _ = frame.shape
# out = cv2.VideoWriter(video_path_out, cv2.VideoWriter_fourcc(*'MP4V'), int(cap.get(cv2.CAP_PROP_FPS)), (W, H))

# model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'last.pt')

# # Load a model
# model = YOLO(model_path)  # load a custom model

# threshold = 0.5

# while ret:

#     results = model(frame)[0]

#     for result in results.boxes.data.tolist():
#         x1, y1, x2, y2, score, class_id = result

#         if score > threshold:
#             cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
#             cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
#                         cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

#     out.write(frame)
#     print('Frame written')
#     ret, frame = cap.read()

# cap.release()
# out.release()
# cv2.destroyAllWindows()


class BottleDetector:
    def __init__(self, model_path, topicName):
        self.model = YOLO(model_path)

        NodeName = "bottle_detector_node"

        # make sure that the same name is used in the soufce file of the publisher
        self.topicName = topicName

        # initialize the subscriber node
        # anonymous = True means that a random number is added to the subscriber node name
        rospy.init_node(NodeName, anonymous=True)
        
        # specify the topic name, type of the message will receive, and the name of the callback function 
        rospy.Subscriber(topicName,Image, self.callback)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("bottle_detector", Image, queue_size=10)

    def callback(self, msg):

        rospy.loginfo("received a video message/frame")

        # convert from cv_bridge to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        # receive the cv2 image
        # path_img = "output.png"
        # cv2.imwrite(path_img, convertedFrame)
        # cv2.imshow("camera", cv_image)

        cv_image = cv_image[:, :, :3] # Remove alpha channel
        # print(cv_image.shape)
        result = self.model(cv_image, stream=True, save=False, show=False)

        img = cv_image.copy()
        # image = cv2.imread(cv_image)
        # cv_image.flags.writeable = True
        predicted = False
        # for prediction in result:
        #     json_object = json.loads(prediction.tojson())
        #     if len(json_object) != 0:
        #         for object in json_object:
        #             convertedFrame = cv2.rectangle(img, 
        #                                         (int(object['box']['x1']), int(object['box']['y1'])), 
        #                                         (int(object['box']['x2']), int(object['box']['y2'])), 
        #                                         (0, 255, 0), 2)
        #             predicted = True
                    
        # print(result)
        for r in result:
            boxes = r.boxes
            # print(boxes)
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = x2-x1, y2-y1
                # cvzone.cornerRect(cv_image, (x1, y1, w, h))
                convertedFrame = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # conf = math.ceil((box.conf[0]*100))/100
                # cv2.putText(cv_image, f'{r.names[int(box.get_class())]}: {conf}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                predicted = True

        if predicted:
            # convert the cv2 image to a ROS image
            msg = self.bridge.cv2_to_imgmsg(convertedFrame)
            # publish the ROS image
            self.pub.publish(msg)

        # if predicted:
        #     cv2.imshow("detector", convertedFrame)

        #     cv2.waitKey(1)



    def detect(self, frame):
        return self.model(frame)[0]

    def draw_boxes(self, frame, results, threshold=0.5):
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

        return frame

    def detect_and_draw(self, frame):
        results = self.detect(frame)
        return self.draw_boxes(frame, results)

# model = YOLO(r'/home/garvish/rover_ws/src/rsx-rover/rover/vision/bottle_detector.pt') # Change this path

# while True:
#     # res = model.predict(source=r'/home/garvish/Plastic Bottle Image Dataset/test/images/-2_jpg.rf.56c6648de860249fe106b20ce690a650.jpg', show=True)
#     # res = model.predict(source=r'/home/garvish/Plastic Bottle Image Dataset/img1.jpg', show=True)
#     res = model.predict(source=0, show=True)


if __name__ == '__main__':
    path = os.path.dirname(os.path.abspath(__file__))
    # print(path)
    model_path = os.path.join(path, 'bottle_detector.pt')
    # topicName = "/zed_node/rgb/image_rect_color"
    # detector = BottleDetector(model_path, topicName)
    # rospy.spin()
    # cv2.destroyAllWindows()

    model = YOLO(model_path)
    # cap = cv2.VideoCapture(0)
    # ret, frame = cap.read()
    # # cv2.imshow("image", frame)
    # while ret:     
    #         results = model(frame)[0]
    
    #         for result in results.boxes.data.tolist():
    #             x1, y1, x2, y2, score, class_id = result
    
    #             if score > 0.5:
    #                 cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
    #                 cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
    
    #         cv2.imshow("detector", frame)
    #         cv2.waitKey(1)

    while True:
        # res = model.predict(source=r'/home/garvish/Plastic Bottle Image Dataset/test/images/-2_jpg.rf.56c6648de860249fe106b20ce690a650.jpg', show=True)
        # res = model.predict(source=r'/home/garvish/rover_ws/src/rsx-rover/rover/vision/bottle_detection/IMG_3915.jpg', show=True)
        res = model.predict(source=0, show=True)
        



