#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import os
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from ultralytics import YOLO

bridge = CvBridge()

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.image_topic = "/zed_node/rgb/image_rect_color"
        self.info_topic = "/zed_node/rgb/camera_info"
        # self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        # self.cam_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback)
        # self.mallet_pub = rospy.Publisher('mallet_detected', Bool, queue_size=1)
        # self.waterbottle_pub = rospy.Publisher('waterbottle_detected', Bool, queue_size=1)
        # self.bbox_pub = rospy.Publisher('object/bbox', Float64MultiArray, queue_size=10)
        # self.vis_pub = rospy.Publisher('vis/object_detections', Image, queue_size=10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, 10)
        self.mallet_pub = self.create_publisher(Bool, 'mallet_detected', 10)
        self.waterbottle_pub = self.create_publisher(Bool, 'waterbottle_detected', 10)
        self.bbox_pub = self.create_publisher(Float64MultiArray, 'object/bbox', 10)
        self.vis_pub = self.create_publisher(Image, 'vis/object_detections', 10)
        
        self.K = None
        self.D = None
        script_dir=os.path.dirname(os.path.abspath(__file__))
        model_path=os.path.join(script_dir, 'best.pt')
        self.model = YOLO(model_path)  # Load YOLO model
        self.model.conf = 0.5  # Set confidence threshold

        # Mapping class indices to object names (update as per your model's training labels)
        self.class_map = {0: "mallet", 1: "waterbottle"}  # Adjust indices based on your model's label order

    def info_callback(self, info_msg):
        self.D = np.array(info_msg.D)
        self.K = np.array(info_msg.K).reshape(3, 3)

    def image_callback(self, ros_image):
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            
            self.get_logger().error(f"Failed to convert ROS image to CV2: {e}")
            return
        self.detect_objects(cv_image)

    def detect_objects(self, img):
        results = self.model(img)  # Run YOLO detection
      #  print("RESULTS: ", results)
        detections = results[0].boxes.xyxy.cpu().numpy()  
        mallet_found = False
        waterbottle_found = False

        for *bbox, conf, cls in detections:
            cls = int(cls)  # Convert class to integer
            if cls not in self.class_map:
                continue  # Skip unrecognized classes

            obj_name = self.class_map[cls]
            x1, y1, x2, y2 = map(int, bbox)
            width, height = x2 - x1, y2 - y1
            area = width * height

            # Publish bounding box
            bbox_data = Float64MultiArray(data=[x1, y1, x2, y2, area, cls])
            self.bbox_pub.publish(bbox_data)

            # Draw bounding box and label on image
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{obj_name} {conf:.2f}"
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Mark objects as found
            if obj_name == "mallet":
                print("MALLET FOUND")
                mallet_found = True
            elif obj_name == "waterbottle":
                print("WATERBOTTLE FOUND")
                waterbottle_found = True

        # Publish detection status
        self.mallet_pub.publish(Bool(data=mallet_found))
        self.waterbottle_pub.publish(Bool(data=waterbottle_found))

        # Publish visualized image
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.vis_pub.publish(img_msg)


def main():
    rclpy.init()
    object_detector = ObjectDetectionNode()
    rclpy.spin(object_detector)


if __name__ == "__main__":
    main()