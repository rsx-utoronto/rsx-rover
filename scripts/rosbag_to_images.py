#!/usr/bin/python3

import rosbag
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import argparse
import time
from pathlib import Path

class RosbagToImages:
    def __init__(self, bag_path, output_dir, image_topic, extract_frequency=1.0, max_images=None):
        """
        Extract images from a rosbag file for YOLO training
        
        Args:
            bag_path (str): Path to the rosbag file
            output_dir (str): Directory to save extracted images
            image_topic (str): ROS topic name containing image data
            extract_frequency (float): How many images to extract per second
            max_images (int, optional): Maximum number of images to extract
        """
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.image_topic = image_topic
        self.extract_frequency = extract_frequency
        self.max_images = max_images
        self.bridge = CvBridge()
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
    
    def extract_images(self):
        """Extract images from the rosbag file and save them directly"""
        print(f"Opening bag file: {self.bag_path}")
        try:
            bag = rosbag.Bag(self.bag_path, 'r')
        except Exception as e:
            print(f"Error opening bag file: {e}")
            return False
        
        # Get information about the bag
        info = bag.get_type_and_topic_info()
        if self.image_topic not in info.topics:
            print(f"Error: Topic '{self.image_topic}' not found in bag file.")
            print(f"Available topics: {info.topics.keys()}")
            bag.close()
            return False
        
        # Calculate time interval between extracted frames
        time_interval = 1.0 / self.extract_frequency
        
        # Extract images
        print(f"Extracting images from topic '{self.image_topic}'...")
        count = 0
        last_extract_time = None
        
        for topic, msg, t in bag.read_messages(topics=[self.image_topic]):
            # Skip frames to match extract_frequency
            current_time = t.to_sec()
            if last_extract_time is not None and (current_time - last_extract_time) < time_interval:
                continue
            
            last_extract_time = current_time
            
            try:
                # We still need minimal conversion to save the image data
                # This is the simplest approach to get raw image data
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                
                # Save the image with minimal processing
                filename = os.path.join(self.output_dir, f"frame_{count:06d}.jpg")
                cv2.imwrite(filename, cv_image)
                
                count += 1
                if count % 10 == 0:
                    print(f"Extracted {count} images")
                
                # Stop if we've reached the maximum number of images
                if self.max_images is not None and count >= self.max_images:
                    break
                    
            except CvBridgeError as e:
                print(f"Error saving image: {e}")
        
        bag.close()
        print(f"Extraction complete. Saved {count} images to {self.output_dir}")
        return True

    def prepare_for_training(self):
        """Prepare extracted images for YOLO training by organizing into directory structure"""
        # This integrates with your existing data preparation workflow
        print("To prepare these images for training:")
        print(f"1. Update the image_dir in yamlfile_generator.py to: {self.output_dir}")
        print("2. Run yamlfile_generator.py to split the data and create YAML config")
        print("3. Use YoloV8.py or Yolov8_test.py to train the model")

def record_webcam_to_bag(output_bag, duration=60, device_id=0, topic_name="/webcam/image_raw"):
    """
    Record webcam feed directly to a rosbag file
    
    Args:
        output_bag (str): Path to save the rosbag file
        duration (int): Recording duration in seconds
        device_id (int): Webcam device ID
        topic_name (str): Topic name to use for the images
    """
    # Initialize ROS node
    rospy.init_node('webcam_recorder', anonymous=True)
    
    # Open webcam
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print(f"Error: Could not open webcam (device ID: {device_id})")
        return False
    
    # Get webcam properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Webcam settings: {width}x{height} @ {fps}fps")
    
    # Create bag file
    bag = rosbag.Bag(output_bag, 'w')
    bridge = CvBridge()
    
    # Record for specified duration
    start_time = time.time()
    frame_count = 0
    
    print(f"Recording webcam to {output_bag} for {duration} seconds...")
    
    try:
        while (time.time() - start_time) < duration:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Convert frame to ROS message
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera"
                
                # Write to bag
                bag.write(topic_name, ros_image, ros_image.header.stamp)
                
                frame_count += 1
                if frame_count % 30 == 0:
                    print(f"Recorded {frame_count} frames, elapsed time: {time.time() - start_time:.1f}s")
                    
            except CvBridgeError as e:
                print(f"Error converting frame: {e}")
    
    except KeyboardInterrupt:
        print("Recording stopped by user")
    
    finally:
        # Clean up
        bag.close()
        cap.release()
        print(f"Recording complete. Saved {frame_count} frames to {output_bag}")
    
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images from rosbag or record webcam to rosbag")
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Extract command
    extract_parser = subparsers.add_parser("extract", help="Extract images from rosbag")
    extract_parser.add_argument("--bag", required=True, help="Path to the input rosbag file")
    extract_parser.add_argument("--output", required=True, help="Directory to save extracted images")
    extract_parser.add_argument("--topic", required=True, help="Image topic to extract frames from")
    extract_parser.add_argument("--freq", type=float, default=1.0, help="Extraction frequency (frames per second)")
    extract_parser.add_argument("--max", type=int, help="Maximum number of frames to extract")
    
    # Record command
    record_parser = subparsers.add_parser("record", help="Record webcam to rosbag")
    record_parser.add_argument("--output", required=True, help="Path to save the output rosbag file")
    record_parser.add_argument("--duration", type=int, default=60, help="Recording duration in seconds")
    record_parser.add_argument("--device", type=int, default=0, help="Webcam device ID")
    record_parser.add_argument("--topic", default="/webcam/image_raw", help="Topic name for recorded images")
    
    args = parser.parse_args()
    
    if args.command == "extract":
        extractor = RosbagToImages(
            args.bag, 
            args.output, 
            args.topic, 
            extract_frequency=args.freq, 
            max_images=args.max
        )
        if extractor.extract_images():
            extractor.prepare_for_training()
    
    elif args.command == "record":
        record_webcam_to_bag(
            args.output,
            duration=args.duration,
            device_id=args.device,
            topic_name=args.topic
        )
    
    else:
        parser.print_help() 