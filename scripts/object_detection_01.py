#!/usr/bin/python3

import rospy
import cv2
import torch
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray, Bool
from ultralytics import YOLO

bridge = CvBridge()

class ObjectDetectionNode():
    def __init__(self):
        # ROS topics and subscribers/publishers
        self.image_topic = "/zed_node/rgb/image_rect_color"
        self.info_topic = "/zed_node/rgb/camera_info"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback)
        self.mallet_pub = rospy.Publisher('mallet_detected', Bool, queue_size=1)
        self.waterbottle_pub = rospy.Publisher('waterbottle_detected', Bool, queue_size=1)
        self.bbox_pub = rospy.Publisher('object/bbox', Float64MultiArray, queue_size=10)
        self.vis_pub = rospy.Publisher('vis/object_detections', Image, queue_size=10)

        self.K = None
        self.D = None

        # Load YOLO model (adjust the weights file path as needed)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'model.pt')
        self.model = YOLO(model_path)
        self.model.conf = 0.5  # Set confidence threshold

        # Mapping class indices to object names
        self.class_map = {0: "mallet", 1: "waterbottle"}

        # Initialize MiDaS depth estimator
        self.midas = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True)
        self.midas.eval()

        # Define device before using it anywhere else
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas.to(self.device)

    def info_callback(self, info_msg):
        self.D = np.array(info_msg.D)
        self.K = np.array(info_msg.K).reshape(3, 3)

    def image_callback(self, ros_image):
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert ROS image to CV2: {e}")
            return

        # Compute depth map using MiDaS and enhance the image accordingly
        depth_map = self.process_depth(cv_image)
        processed_frame = self.enhance_detection(cv_image, depth_map)

        # Run object detection on the enhanced image
        self.detect_objects(processed_frame)

    def process_depth(self, frame):
        """
        Generate a depth map from the input frame using MiDaS.
        The frame is resized for the model and the output depth is
        interpolated back to the original frame size and normalized.
        """
        # Convert to RGB and resize for MiDaS (MiDaS_small expects 384x384)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_size = (384, 384)
        img_resized = cv2.resize(img_rgb, input_size)
        img_tensor = torch.from_numpy(img_resized).permute(2, 0, 1).unsqueeze(0).float()
        img_tensor = img_tensor.to(self.device)

        with torch.no_grad():
            depth = self.midas(img_tensor)

        # Resize depth map back to original frame size
        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(1),
            size=frame.shape[:2],
            mode="bilinear",
            align_corners=False
        ).squeeze().cpu().numpy()

        # Normalize depth values between 0 and 1
        depth_normalized = cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)
        return depth_normalized

    def enhance_detection(self, frame, depth_map):
        """
        Enhance the image for object detection using depth information.
        The enhancement includes CLAHE on the luminance channel and a
        depth-aware sharpening filter.
        """
        # Normalize frame to float [0,1]
        frame_float = frame.astype(np.float32) / 255.0

        # Apply CLAHE to the luminance channel
        lab = cv2.cvtColor(frame_float, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l_clahe = clahe.apply((l * 255).astype(np.uint8)).astype(np.float32) / 255.0
        enhanced_lab = cv2.merge((l_clahe, a, b))
        enhanced = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)

        # Apply a sharpening filter
        sharpen_kernel = np.array([[0, -1, 0],
                                   [-1, 5, -1],
                                   [0, -1, 0]])
        sharpened = cv2.filter2D(enhanced, -1, sharpen_kernel)

        # Use the depth map as a weight to combine the original enhanced and sharpened images
        depth_weight = np.clip(depth_map * 1.5, 0, 1)[..., np.newaxis]
        processed = enhanced * 0.7 + sharpened * 0.3 * depth_weight
        processed = np.clip(processed, 0, 1)

        return (processed * 255).astype(np.uint8)

    def detect_objects(self, img):
        if self.model is None:
            rospy.logerr("YOLO model is not loaded.")
            return

        # Run YOLO inference on the enhanced image
        results = self.model(img)

        for result in results:
            detections = result.boxes  # Contains bounding boxes, scores, and class predictions
            mallet_found = False
            waterbottle_found = False

            for box in detections:
                # Extract bounding box coordinates, confidence, and class index
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls = int(box.cls[0])
                if cls not in self.class_map:
                    continue
                obj_name = self.class_map[cls]
                width, height = x2 - x1, y2 - y1
                area = width * height

                # Publish bounding box data
                bbox_data = Float64MultiArray(data=[x1, y1, x2, y2, area, cls])
                self.bbox_pub.publish(bbox_data)

                # Draw bounding box and label on the image
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{obj_name} {conf:.2f}"
                cv2.putText(img, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Mark objects as found for publishing detection status
                if obj_name == "mallet":
                    mallet_found = True
                elif obj_name == "waterbottle":
                    waterbottle_found = True

            # Publish detection status for each object
            self.mallet_pub.publish(Bool(data=mallet_found))
            self.waterbottle_pub.publish(Bool(data=waterbottle_found))

            # Publish the visualized image with detections
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.vis_pub.publish(img_msg)

def main():
    rospy.init_node('object_detector', anonymous=True)
    ObjectDetectionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
