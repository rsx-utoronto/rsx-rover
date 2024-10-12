#!/usr/bin/env python3

import sys
from enum import Enum
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QComboBox, QGridLayout
from PyQt5.QtCore import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter

class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class VelocityControl:
    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=10)

    def send_velocity(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)
        print(f"Publishing to /drive: linear_x = {linear_x}, angular_z = {angular_z}")  # Debug print

class Direction: 
    def __init__(self):
        self.LinX =0
        self.AngleZ = 0

class Joystick(QWidget):
    def __init__(self, velocity_control, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(100, 100)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50
        self.direction = Direction()
        self.velocity_control = velocity_control  # Pass the VelocityControl object

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width()/2, self.height()/2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        linX = 0
        angleZ = 0

        # Calculate linear velocity and angular velocity
        if 0 <= angle < 180:
            linX = distance
        elif 180 <= angle < 360:
            linX = -distance

        if 0 <= angle < 90:
            angleZ = angle / 45
        elif 270 <= angle < 360:
            angleZ = (angle - 270) / 45
        elif 90 <= angle < 180:
            angleZ = -(angle - 90) / 45
        else:
            angleZ = -(angle - 180) / 45

        self.direction.LinX = linX
        self.direction.AngleZ = angleZ

        # Send velocity to the VelocityControl object
        self.velocity_control.send_velocity(self.direction.LinX, self.direction.AngleZ)

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        self.joystickDirection()  # Call the function to update the velocity

        

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


class RoverGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 800, 600)

        # Camera feed widget
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(400, 300)

        # Camera toggle
        self.camera_selector = QComboBox(self)
        self.camera_selector.addItem("Zed (front) camera")
        self.camera_selector.addItem("butt camera")
        self.camera_selector.currentIndexChanged.connect(self.switch_camera)

        # ROS functionality
        self.camera_feed = CameraFeed(self.camera_label)
        self.velocity_control = VelocityControl()  # Initialize the VelocityControl object

        # Joystick widget with velocity control
        self.joystick = Joystick(self.velocity_control)

        # Layout setup
        layout = QGridLayout()
        layout.addWidget(QLabel("Camera Feed"), 0, 0)
        layout.addWidget(self.camera_label, 1, 0, 1, 2)
        layout.addWidget(QLabel("Select Camera"), 2, 0)
        layout.addWidget(self.camera_selector, 2, 1)
        layout.addWidget(self.joystick, 3, 0, 1, 2)

        self.setLayout(layout)

        # Start ROS in a separate thread
        self.ros_thread = ROSWorker()
        self.ros_thread.start()

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)


if __name__ == '__main__':
    rospy.init_node('rover_gui', anonymous=False)  # Initialize ROS node
    app = QApplication(sys.argv)
    gui = RoverGUI()
    gui.show()
    sys.exit(app.exec_())

