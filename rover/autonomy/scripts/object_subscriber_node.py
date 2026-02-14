#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from ultralytics import YOLO
import torch
import os
import yaml
import threading 
from std_msgs.msg import String
from rover.msg import MissionState
file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        if sm_config.get("realsense_detection"):
            self.image_topic = sm_config.get("realsense_detection_image_topic") 
            self.info_topic = sm_config.get("realsense_detection_info_topic")
        else:
            self.image_topic = sm_config.get("zed_detection_image_topic") 
            self.info_topic = sm_config.get("zed_detection_info_topic")    
            
        self.state_topic = "state"
        self.curr_state = "Start"
        t = time.time()
        # while (time.time() - t) < 2:
        #     #print("Passing time") 
        #     pass
        self.bridge = CvBridge()

        print("Object Detection Node Initialized\n\n\n\n")


        self.image_sub = self.create_subscription( Image,self.image_topic, self.image_callback,10)
        self.cam_info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback,10)
        self.state_sub = self.create_subscription(String, self.state_topic, self.state_callback, 10)
        self.mallet_pub = self.create_publisher(Bool, 'mallet_detected', 1)
        self.waterbottle_pub = self.create_publisher(Bool, 'waterbottle_detected', 1)
        self.bbox_pub = self.create_publisher(Float64MultiArray, 'object/bbox', 10)
        self.vis_pub = self.create_publisher(Image, 'vis/object_detections', 10)
        self.mission_state_pub = self.create_publisher(MissionState, 'mission_state', 10)
        self.create_subscription(MissionState,'mission_state',self.mission_state_callback, 10)
        self.mallet_found = False
        self.waterbottle_found = False
        self.last_cv_image = None
        self.K = None
        self.D = None
        script_dir=os.path.dirname(os.path.abspath(__file__))
        model_path=os.path.join(script_dir, 'model_v1.pt')
        self.model = YOLO(model_path)  # Load YOLO model
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)
        self.HALF = torch.cuda.is_available()
        self.CLASSES = list(self.model.names.values())
        self._nav_thread = None
        
        #self.model = YOLO('best.pt')  #Load YOLO model
        self.conf = 0.5  # Set confidence threshold

        # Mapping class indices to object names (update as per your model's training labels)
        # self.class_map = {0: "mallet", 1: "waterbottle"}  # Adjust indices based on your model's label order

    def info_callback(self, info_msg):
        self.D = np.array(info_msg.d)
        self.K = np.array(info_msg.k).reshape(3, 3)

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.last_cv_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert ROS image to CV2: {e}")
            return
        if self.curr_state == "OBJ1" or self.curr_state == "OBJ2":
            self.detect_objects(cv_image)
    
    def mission_state_callback(self, msg):
        self.mission_state_msg = msg.state
        if self.mission_state_msg == "START_GS_TRAV":
            if getattr(self, 'last_cv_image') is not None:
                if self._nav_thread is None or not self._nav_thread.is_alive():
                    self._nav_thread = threading.Thread(target=self.detect_objects, args=(self.last_cv_image,), daemon=True)
                    self._nav_thread.start()
                
            else: 
                self.get_logger().info("ar_detection_node: No image received yet for AR detection.")

    def state_callback(self, state):
        self.curr_state = state.data

    def detect_objects(self, img):
        print("In detect objects")
        if self.model==None: 
            print("Model is NULL")
        
        msg=MissionState()
            
        # results = self.model(img, verbose=False)  # Run YOLO detection
        results = self.model.predict(
            source=img, 
            conf=self.conf,
            device=self.device,
            half=self.HALF,
            verbose=False
        )
        # print("Object detection detect_objects")
        for result in results:
            # print("actual results", results)
            detections = result.boxes  # This contains the bounding boxes, scores, and class predictions
            mallet_found = False
            waterbottle_found = False
            for box in detections:
                # Extract bbox coordinates, confidence, and class
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                conf = box.conf[0]  # Confidence score
                cls = int(box.cls[0])  # Class index
                if cls not in range(len(self.CLASSES)):
                    continue  # Skip unrecognized classes
                obj_name = self.CLASSES[cls]
                width, height = x2 - x1, y2 - y1
                area = width * height
                # Publish bounding box
                bbox_data = Float64MultiArray(data=[x1, y1, x2, y1, x1, y2, x2, y2])
                self.bbox_pub.publish(bbox_data)
                # Draw bounding box and label on image
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{obj_name} {conf:.2f}"
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Mark objects as found
                if obj_name == "mallet":
                    print("Mallet found")
                    mallet_found = True
                    msg.state="OBJ_FOUND"
                    self.mission_state_pub.publish(msg)      
                elif obj_name == "waterbottle":
                    print("Waterbottle found")
                    waterbottle_found = True
                    msg.state="OBJ_FOUND"
                    self.mission_state_pub.publish(msg)      
                else:
                    print("No object detected")
                    msg.state="OBJ_NOT_FOUND"
                    self.mission_state_pub.publish(msg)      
            # Publish detection status
            self.mallet_pub.publish(Bool(data=mallet_found))
            self.waterbottle_pub.publish(Bool(data=waterbottle_found))
            # Publish visualized image
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.vis_pub.publish(img_msg)
              

def main():
    import rclpy
    rclpy.init()
    object_detector = ObjectDetectionNode()
    try:
        rclpy.spin(object_detector)
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()