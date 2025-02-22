import rospy
import cv2
from PyQt6.QtCore import pyqtSignal, QObject
from PyQt6.QtGui import QImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSVideoSubscriber(QObject):
    frame_received = pyqtSignal(QImage)

    def __init__(self, topic_name):
        super().__init__()
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic_name, Image, self.callback)

    def callback(self, msg):
        # Convert the ROS Image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Convert the CV2 image to RGB format for PyQt
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Convert the image to a format suitable for PyQt (QImage)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        
        # Emit the converted QImage to update the GUI
        self.frame_received.emit(qt_image)
