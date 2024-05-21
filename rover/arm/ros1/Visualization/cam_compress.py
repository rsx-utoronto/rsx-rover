#!/usr/bin/env python3
import cv2
import sys
import numpy
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# # argvs=sys.argv
# # if (len(argvs) != 2):
# #     print 'Usage: # python %s imagefilename' % argvs[0]
# #     quit()
 
# # imagefilename = argvs[1]
# try:
#      vid=cv2.VideoCapture("/dev/video6")
# except:
#      print('faild to load pic')
#      quit()

# #encode to jpeg format
# #encode param image quality 0 to 100. default:95
# #if you want to shrink data size, choose low image quality.
# while(True):
#     ret, img = vid.read()  
#     # cv2.imshow('Source Image',img)
#     # cv2.waitKey(0)      
#     encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),10]
#     result,encimg=cv2.imencode('.jpg',img,encode_param)
#     if False==result:
#         print('could not encode image!')
#         quit()

#     # #decode from jpeg format
#     decimg=cv2.imdecode(encimg,1)
#     cv2.imshow('Source Image',img)
#     cv2.imshow('Decoded image',decimg)
#     cv2.waitKey(0)
#     # cv2.destroyAllWindows()
#     # publish(image data) 




# import cv2
# import sys
# import numpy
# import rospy
# from sensor_msgs.msg import Image

# argvs=sys.argv
# if (len(argvs) != 2):
#     print 'Usage: # python %s imagefilename' % argvs[0]
#     quit()

rospy.init_node("image_encoder", anonymous=True)
pub = rospy.Publisher('encoded_image_topic', Image, queue_size=1000)
 
# imagefilename = argvs[1]
try:
     vid=cv2.VideoCapture(0)
except:
     print('faild to load pic')
     quit()

bridge = CvBridge()
#encode to jpeg format
#encode param image quality 0 to 100. default:95
#if you want to shrink data size, choose low image quality.
while(True):
    ret, img = vid.read()  
    # cv2.imshow('Source Image',img)
    # cv2.waitKey(0)      
    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),10]
    result,encimg=cv2.imencode('.jpg',img,encode_param)
    if False==result:
        print('could not encode image!')
        # quit()

    # #decode from jpeg format
    # decimg=cv2.imdecode(encimg,1)
    # cv2.imshow('Source Image',img)
    # cv2.imshow('Decoded image',decimg)
    # cv2.waitKey(0)
    
    # cv2.destroyAllWindows()
    pub.publish(bridge.cv2_to_imgmsg(encimg, 'mono8'))
                    