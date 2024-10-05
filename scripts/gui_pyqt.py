#!/usr/bin/env python3

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QComboBox, QGridLayout
from PyQt5.QtCore import QTimer, QThread
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap


class ROSWorker(QThread):
    """
    A QThread that runs rospy.spin() in the background to keep ROS processing messages.
    """
    def __init__(self):
        super(ROSWorker, self).__init__()

    def run(self):
        rospy.spin()  # Run ROS event loop in a separate thread


class CameraFeed:
    def __init__(self, widget):
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera/color/image_raw", Image, self.callback2)
        self.current_image = None
        self.current_camera = 1  # 1 for camera1, 2 for camera2
        self.label = widget

    def callback1(self, data):
        if self.current_camera == 1:
            self.update_image(data)

    def callback2(self, data):
        if self.current_camera == 2:
            self.update_image(data)

    def update_image(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Convert BGR to RGB for QImage
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qimg = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.label.setPixmap(pixmap)

    def switch_camera(self, camera_index):
        self.current_camera = camera_index


class VelocityControl:
    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=10)

    def send_velocity(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)


class RoverGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 800, 600)

        # Camera feed widget
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(400, 300)

        # Velocity control buttons
        self.forward_button = QPushButton("Forward", self)
        self.forward_button.clicked.connect(self.move_forward)

        self.left_button = QPushButton("Left", self)
        self.left_button.clicked.connect(self.move_left)

        self.right_button = QPushButton("Right", self)
        self.right_button.clicked.connect(self.move_right)

        self.backward_button = QPushButton("Backward", self)
        self.backward_button.clicked.connect(self.move_backward)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.clicked.connect(self.stop)

        # Camera toggle
        self.camera_selector = QComboBox(self)
        self.camera_selector.addItem("Zed (front) camera")
        self.camera_selector.addItem("butt camera")
        self.camera_selector.currentIndexChanged.connect(self.switch_camera)

        # Layout setup
        layout = QGridLayout()
        layout.addWidget(QLabel("Camera Feed"), 0, 0)
        layout.addWidget(self.camera_label, 1, 0, 1, 2)
        layout.addWidget(QLabel("Select Camera"), 2, 0)
        layout.addWidget(self.camera_selector, 2, 1)

        # Velocity Control Layout
        layout.addWidget(self.forward_button, 4, 1)
        layout.addWidget(self.left_button, 5, 0)
        layout.addWidget(self.stop_button, 5, 1)
        layout.addWidget(self.right_button, 5, 2)
        layout.addWidget(self.backward_button, 6, 1)

        self.setLayout(layout)

        # ROS functionality
        self.camera_feed = CameraFeed(self.camera_label)
        self.velocity_control = VelocityControl()

        # Start ROS in a separate thread
        self.ros_thread = ROSWorker()
        self.ros_thread.start()

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)

    def move_forward(self):
        self.velocity_control.send_velocity(1.0, 0.0)

    def move_left(self):
        self.velocity_control.send_velocity(0.0, 1.0)

    def move_right(self):
        self.velocity_control.send_velocity(0.0, -1.0)

    def move_backward(self):
        self.velocity_control.send_velocity(-1.0, 0.0)

    def stop(self):
        self.velocity_control.send_velocity(0.0, 0.0)


if __name__ == '__main__':
    rospy.init_node('rover_gui', anonymous=False)  # Initialize ROS node
    app = QApplication(sys.argv)
    gui = RoverGUI()
    gui.show()
    sys.exit(app.exec_())
