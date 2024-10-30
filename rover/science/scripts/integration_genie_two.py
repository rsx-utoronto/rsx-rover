#! /usr/bin/env python3

import numpy
import rospy 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge 
import math
import numpy as np
import sys
import ctypes
import time
from PIL import Image
import pygigev

# Get the common support code for the GigE-V Framework for Linux
# (Change this if directory structure is changed).
sys.path.append("../gigev_common")

#=======================================================================
# Utilties
def ipAddr_from_string(s):
  "Convert dotted IPv4 address to integer."
  return reduce(lambda a,b: a<<8 | b, map(int, s.split(".")))

def ipAddr_to_string(ip):
  "Convert 32-bit integer to dotted IPv4 address."
  return ".".join(map(lambda n: str(ip>>n & 0xFF), [24,16,8,0]))

#
# The basic program
def main():
	# Initialize the API
	pygigev.GevApiInitialize()

	# Allocate a maximum number of camera info structures.
	maxCameras = 16
	numFound = (ctypes.c_uint32)(0)
	camera_info = (pygigev.GEV_CAMERA_INFO * maxCameras)()

	# Get the camera list 
	status = pygigev.GevGetCameraList( camera_info, maxCameras, ctypes.byref(numFound) )
	if ( status != 0  ):
		print("Error ", status,"getting camera list - exitting")
		quit()

	if (numFound.value == 0):
		print("No cameras found - exitting")
		quit()
		
	# Proceed
	print(numFound.value," Cameras found" )
	for camIndex in range(numFound.value):
		print("ip = ", ipAddr_to_string(camera_info[camIndex].ipAddr))
	
	# Select the first camera and open it.
	camIndex = 0
	print("Opening camera #", camIndex)
	handle = (ctypes.c_void_p)()
	status = pygigev.GevOpenCamera( camera_info[camIndex], pygigev.GevExclusiveMode, ctypes.byref(handle))

	# Get the payload parameters 
	print("Getting payload information :")
	payload_size = (ctypes.c_uint64)()
	pixel_format = (ctypes.c_uint32)()	
	status = pygigev.GevGetPayloadParameters( handle, ctypes.byref(payload_size), ctypes.byref(pixel_format) )
	pixel_format_unpacked = pygigev.GevGetUnpackedPixelType( pixel_format );
	print("status :", status," payload_size : ", payload_size.value, " pixel_format = ", hex(pixel_format.value), " pixel_format_unpacked = ",hex(pixel_format_unpacked ))

	# Get the Width and Height (extra information)
	feature_strlen = (ctypes.c_int)(pygigev.MAX_GEVSTRING_LENGTH)
	unused = (ctypes.c_int)(0)
	if (sys.version_info > (3, 0)):
		width_name = b'Width'
		height_name = b'Height'
	else:
		width_name = "Width"
		height_name = "Height"

	width_str = ((ctypes.c_char)*feature_strlen.value)()
	height_str = ((ctypes.c_char)*feature_strlen.value)()
	status = pygigev.GevGetFeatureValueAsString( handle, width_name, unused, feature_strlen, width_str)
	status = pygigev.GevGetFeatureValueAsString( handle, height_name, ctypes.byref(unused), feature_strlen, height_str)

	print("status :", status," Width : ", width_str.value, " Height = ", height_str.value )
	
	# Allocate buffers to store images in (2 here). 
	# (Handle cases where image is larger than payload due to pixel unpacking)
	numBuffers = 1 #changed to 1 instead of 2
	print(" Allocate ",numBuffers," buffers :")
	buffer_addresses = ((ctypes.c_void_p)*numBuffers)()
	
	bufsize = payload_size.value
	bufsize_unpacked = int(width_str.value) * int(height_str.value) * pygigev.GevGetPixelSizeInBytes( pixel_format_unpacked );
	if (bufsize_unpacked > bufsize):
		bufsize = bufsize_unpacked
	print(" Using bufsize = ", bufsize);

	for bufIndex in range(numBuffers):
		temp = ((ctypes.c_char)*bufsize)()
		buffer_addresses[bufIndex] = ctypes.cast( temp, ctypes.c_void_p)
		print(" buffer_addresses[",bufIndex,"] = ",hex(buffer_addresses[bufIndex]))
	
	# Initialize a transfer (Asynchronous cycling)
	print("Init transfer :")
	status = pygigev.GevInitializeTransfer( handle, pygigev.Asynchronous, payload_size, numBuffers, buffer_addresses)

	# Grab images to fill the buffers 
	numImages = numBuffers
	print("Snap ",numImages," images :")
	status = pygigev.GevStartTransfer(handle, numImages)
	
	# Read the images out
	gevbufPtr = ctypes.POINTER(pygigev.GEV_BUFFER_OBJECT)()
	displayed = 0
		
	for imgIndex in range(numImages):
		tmout = (ctypes.c_uint32)(1000)
		status = pygigev.GevWaitForNextFrame( handle, ctypes.byref(gevbufPtr), tmout.value )
		
		if status == 0:
			# Check img data status
			gevbuf = gevbufPtr.contents
			if gevbuf.status == 0:
				print("Img # ", imgIndex," : id = ", gevbuf.id, " w = ", gevbuf.w, " h = ", gevbuf.h, " address = ", hex(gevbuf.address))
				if ( displayed == 0):
					# Make a PIL image out of this frame (assume 8 bit Mono (also works for 8bit Bayer before decoding))
					displayed = 1
					im_size = ( gevbuf.w , gevbuf.h )
					im_addr = ctypes.cast( gevbuf.address, ctypes.POINTER(ctypes.c_ubyte * gevbuf.recv_size) )
					im = Image.frombuffer( 'L', im_size, im_addr.contents, 'raw', 'L', 0, 1 )
					# Display the image 
					# This creates a new window for each image that will persist even after the program exits!
					#im.show()
			else :
				print("Img # ", imgIndex, " has status = ", gevbuf.status)

	# Free the transfer
	print("Free transfer :")
	status = pygigev.GevFreeTransfer(handle)

	# Close the camera
	print("Close camera :")
	status = pygigev.GevCloseCamera( ctypes.byref(handle) )
	
	# Uninitialize
	pygigev.GevApiUninitialize()

	return im


def cv_ros_cv():
    rospy.init_node('image_publisher')
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    bridge = CvBridge()
    genie_image = main()
    cv_image = cv2.cvtColor(np.array(genie_image), cv2.COLOR_BAYER_BG2BGR)  # Adjust based on the image format    
    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  # Convert to ROS Image message    
    image_pub.publish(ros_image)  # Publish the ROS image

    rospy.sleep(0.1)  # Adjust based on your requirements

if __name__ == '__main__':
    try:
       cv_ros_cv()
    except rospy.ROSInterruptException:
        pass
    
    