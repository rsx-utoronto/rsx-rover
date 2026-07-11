'''
Object Detection with the Intel RealSense D435i Depth Camera.

Code is ROS independent, ideal for quick testing.
Applies a basic pretrained YOLOv8 model on the 
video stream from the RealSense. 
'''

# RealSense SDK Python Wrapper
import pyrealsense2 as rs 

import numpy as np
import keyboard

# YOLOv8 Library
from ultralytics import YOLO

# Realsense interfacing  
pipe = rs.pipeline()
cfg  = rs.config()

# Screen Resolution
WIDTH = 640
HEIGHT = 480

# Enable RGB Stream
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30) 
pipe.start(cfg)

# Import trained model
model = YOLO('yolov8n.pt')

while True:
    
    # Get video stream frame.
    frame = pipe.wait_for_frames()
    color_frame = frame.get_color_frame()

    # Convert to frame to np array.
    color_image = np.asanyarray(color_frame.get_data())
    
    # Inference on image.
    results = model(color_image, show=True, conf=0.4, save=False) 

    # Break.
    if keyboard.is_pressed('q'):
        break

pipe.stop()