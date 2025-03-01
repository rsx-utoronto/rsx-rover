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

# Define the default output directory to be used across the script
DEFAULT_OUTPUT_DIR = "/rover_ws/src/rsx-rover/scripts/new_images"

class RosbagToImages:
    def __init__(self, bag_path, output_dir=None, image_topic=None, extract_frequency=1.0, max_images=None):
        """
        Extract images from a rosbag file for YOLO training
        
        Args:
            bag_path (str): Path to the rosbag file
            output_dir (str): Directory to save extracted images
            image_topic (str): ROS topic name containing image data
            extract_frequency (float): How many images to extract per second
            max_images (int, optional): Maximum number of images to extract
        """
        # Hardcoded values for all parameters
        self.bag_path = "/rover_ws/src/rsx-rover/data/recordings/mallet_recording.bag"
        self.output_dir = "/rover_ws/src/rsx-rover/scripts/new_images"
        self.image_topic = "/zed_node/rgb/image_rect_color"
        self.extract_frequency = 2.0  # 2 frames per second
        self.max_images = 300
        self.bridge = CvBridge()
        
        # Create the main output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"Created output directory: {self.output_dir}")
            
        # Create related directories for different stages of the training pipeline
        self.training_dir = "/rover_ws/src/rsx-rover/scripts/training_images"
        self.validation_dir = "/rover_ws/src/rsx-rover/scripts/validation_images"
        
        # Create these directories if they don't exist
        for directory in [self.training_dir, self.validation_dir]:
            if not os.path.exists(directory):
                os.makedirs(directory)
                print(f"Created directory: {directory}")
                
        # Create subdirectories for images and labels
        for parent_dir in [self.training_dir, self.validation_dir]:
            images_dir = os.path.join(parent_dir, "images")
            labels_dir = os.path.join(parent_dir, "labels")
            
            if not os.path.exists(images_dir):
                os.makedirs(images_dir)
            if not os.path.exists(labels_dir):
                os.makedirs(labels_dir)
        
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

def direct_webcam_capture(output_dir=None, duration=60, device_id=0, capture_freq=1.0, max_images=None):
    """
    Directly capture images from webcam and save them to disk without using ROS
    
    Args:
        output_dir (str): Directory to save the captured images
        duration (int): Recording duration in seconds
        device_id (int): Webcam device ID
        capture_freq (float): How many images to capture per second
        max_images (int, optional): Maximum number of images to capture
    """
    # Use default output directory if none provided
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR
        
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
    
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
    
    # Calculate time between frames
    time_interval = 1.0 / capture_freq
    
    # Start capturing
    start_time = time.time()
    count = 0
    last_capture_time = None
    
    print(f"Capturing images to {output_dir} for {duration} seconds (or until {max_images} images)...")
    
    try:
        while (time.time() - start_time) < duration:
            # Check if we should capture this frame based on frequency
            current_time = time.time()
            if last_capture_time is not None and (current_time - last_capture_time) < time_interval:
                # Wait a bit to avoid maxing out CPU
                time.sleep(0.01)
                continue
            
            # Read frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Save the image
            filename = os.path.join(output_dir, f"frame_{count:06d}.jpg")
            cv2.imwrite(filename, frame)
            
            count += 1
            last_capture_time = current_time
            
            if count % 10 == 0:
                elapsed = time.time() - start_time
                print(f"Captured {count} images, elapsed time: {elapsed:.1f}s")
            
            # Stop if we've reached the maximum number of images
            if max_images is not None and count >= max_images:
                break
                
    except KeyboardInterrupt:
        print("Recording stopped by user")
    
    finally:
        # Clean up
        cap.release()
        print(f"Capture complete. Saved {count} images to {output_dir}")
    
    print("To prepare these images for training:")
    print(f"1. Update the image_dir in yamlfile_generator.py to: {output_dir}")
    print("2. Run yamlfile_generator.py to split the data and create YAML config")
    print("3. Use YoloV8.py or Yolov8_test.py to train the model")
    
    return True

if __name__ == "__main__":
    # Check if any command-line arguments were provided
    import sys
    if len(sys.argv) > 1:
        # If arguments provided, use the argument parser
        parser = argparse.ArgumentParser(description="Extract images from rosbag or record webcam to rosbag")
        subparsers = parser.add_subparsers(dest="command", help="Command to run")
        
        # Extract command
        extract_parser = subparsers.add_parser("extract", help="Extract images from rosbag")
        extract_parser.add_argument("--bag", required=True, help="Path to the input rosbag file")
        extract_parser.add_argument("--output", default=DEFAULT_OUTPUT_DIR, help="Directory to save extracted images")
        extract_parser.add_argument("--topic", required=True, help="Image topic to extract frames from")
        extract_parser.add_argument("--freq", type=float, default=1.0, help="Extraction frequency (frames per second)")
        extract_parser.add_argument("--max", type=int, help="Maximum number of frames to extract")
        
        # Record command
        record_parser = subparsers.add_parser("record", help="Record webcam to rosbag")
        record_parser.add_argument("--output", required=True, help="Path to save the output rosbag file")
        record_parser.add_argument("--duration", type=int, default=60, help="Recording duration in seconds")
        record_parser.add_argument("--device", type=int, default=0, help="Webcam device ID")
        record_parser.add_argument("--topic", default="/webcam/image_raw", help="Topic name for recorded images")
        
        # Direct capture command
        direct_parser = subparsers.add_parser("direct", help="Directly capture webcam images to disk")
        direct_parser.add_argument("--output", default=DEFAULT_OUTPUT_DIR, help="Directory to save captured images")
        direct_parser.add_argument("--duration", type=int, default=60, help="Capture duration in seconds")
        direct_parser.add_argument("--device", type=int, default=0, help="Webcam device ID")
        direct_parser.add_argument("--freq", type=float, default=1.0, help="Capture frequency (frames per second)")
        direct_parser.add_argument("--max", type=int, help="Maximum number of frames to capture")
        
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
        
        elif args.command == "direct":
            direct_webcam_capture(
                output_dir=args.output,
                duration=args.duration,
                device_id=args.device,
                capture_freq=args.freq,
                max_images=args.max
            )
        
        else:
            parser.print_help()
    else:
        # If no arguments provided, automatically run the direct capture
        print("No arguments provided - automatically starting webcam capture")
        print("Press Ctrl+C to stop capturing")
        direct_webcam_capture(
            output_dir=DEFAULT_OUTPUT_DIR,  # Use the default output directory
            duration=300,  # 5 minutes by default
            device_id=0,
            capture_freq=2.0,  # 2 frames per second
            max_images=500
        ) 