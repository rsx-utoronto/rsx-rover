#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import getmicroscopeid # local import

camera_name = "FHD Camera: FHD Camera"
camera_id = getmicroscopeid.get_usb_camera_device(camera_name)

if not camera_id:
    print("WARNING: web camera not found.")
else:
    print(camera_id)

class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam_node")
        
        # Parameters
        # self.compression_quality = rospy.get_param("~compression_quality", 50)  # JPEG quality 0-100
        # self.scale_factor = rospy.get_param("~scale_factor", 0.5)  # Scale resolution by this factor
        
        # Declare the parameters with default values
        self.declare_parameter("compression_quality", 50)  # JPEG quality 0-100
        self.declare_parameter("scale_factor", 0.5)        # Scale resolution by this factor

        # Retrieve the values
        self.compression_quality = self.get_parameter("compression_quality").get_parameter_value().integer_value
        self.scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        
        # Publishers
        # self.pub_compressed = rospy.Publisher("/webcam/compressed", CompressedImage, queue_size=1)
        # self.pub_raw = rospy.Publisher("/webcam", Image, queue_size=1)
        self.pub_compressed = self.create_publisher(CompressedImage, "/webcam/compressed", 1)
        self.pub_raw = self.create_publisher(Image, "/webcam", 1)
        
        # Create bridge object
        self.bridge = CvBridge()
        
        # Get camera feed with MJPG format for better performance
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        if not self.camera.isOpened():
            # rospy.logerr("Failed to open camera")
            self.get_logger().error("Failed to open camera")
            return
            
        self.run()
        
    def run(self):
        while rclpy.ok():
            try:
                ret, frame = self.camera.read()
                
                if ret:
                    # Resize frame to save bandwidth
                    if self.scale_factor != 1.0:
                        h, w = frame.shape[:2]
                        frame = cv2.resize(frame, (int(w*self.scale_factor), int(h*self.scale_factor)))
                    
                    # Publish raw image (still using a smaller resolution)
                    self.pub_raw.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    
                    # Create and publish compressed image
                    compressed_msg = CompressedImage()
                    # compressed_msg.header.stamp = rospy.Time.now()
                    compressed_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = np.array(cv2.imencode(
                        '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.compression_quality]
                    )[1]).tostring()
                    self.pub_compressed.publish(compressed_msg)
                    
                    time.sleep(1/10)
            except Exception as e:
                # rospy.logerr(f"Error processing frame: {str(e)}")
                self.get_logger().error(f"Error processing frame: {str(e)}")

        # Cleanup
        self.camera.release()

if __name__ == "__main__":
    rclpy.init()
    try:
        webcam_node = WebcamNode()
    except rclpy.exceptions.ROSInterruptException:
        pass
