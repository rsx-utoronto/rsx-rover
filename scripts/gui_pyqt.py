#!/usr/bin/env python3

import sys
from enum import Enum
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, QSlider, QHBoxLayout, QVBoxLayout
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
        self.gear = 1

    def set_gear(self, gear):
        self.gear = gear
        print(f"Gear set to: {gear}")

    def send_velocity(self, linear_x, angular_z):
        linear_x *= (self.gear / 10) * 2.5
        angular_z *= (self.gear / 10) * 2.5
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)
        print(f"Publishing to /drive: linear_x = {linear_x}, angular_z = {angular_z}, gear = {self.gear}")

class Joystick(QWidget):
    def __init__(self, velocity_control, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(100, 100)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50
        self.velocity_control = velocity_control

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
        return QPointF(self.width() / 2, self.height() / 2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if limitLine.length() > self.__maxDistance:
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        linX = distance if 0 <= angle < 180 else -distance
        angleZ = 0

        if 0 <= angle < 90:
            angleZ = angle / 45
        elif 270 <= angle < 360:
            angleZ = (angle - 270) / 45
        elif 90 <= angle < 180:
            angleZ = -(angle - 90) / 45
        else:
            angleZ = -(angle - 180) / 45

        self.velocity_control.send_velocity(linX, angleZ)

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.velocity_control.send_velocity(0, 0)
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        self.joystickDirection()

class CameraFeed:
    def __init__(self, widget):
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera/color/image_raw", Image, self.callback2)
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
        self.camera_label.setFixedSize(600, 500)

        # Camera toggle
        self.camera_selector = QComboBox(self)
        self.camera_selector.addItem("Zed (front) camera")
        self.camera_selector.addItem("Butt camera")
        self.camera_selector.currentIndexChanged.connect(self.switch_camera)

        # ROS functionality
        self.camera_feed = CameraFeed(self.camera_label)
        self.velocity_control = VelocityControl()

        # Joystick widget with velocity control
        self.joystick = Joystick(self.velocity_control)

        # Gear slider
        self.gear_slider = QSlider(Qt.Horizontal)
        self.gear_slider.setRange(1, 10)
        self.gear_slider.setTickPosition(QSlider.TicksAbove)
        self.gear_slider.setTickInterval(1)
        self.gear_slider.valueChanged.connect(self.change_gear)

        # Horizontal layout for gear labels and slider
        slider_layout = QVBoxLayout()

        # Add labels above the slider
        label_layout = QHBoxLayout()
        for i in range(1, 11):
            label = QLabel(str(i))
            label.setAlignment(Qt.AlignCenter)
            label_layout.addWidget(label)
        
        slider_layout.addLayout(label_layout)
        slider_layout.addWidget(self.gear_slider)

        # Layout setup
        layout = QGridLayout()
        layout.addWidget(QLabel("Camera Feed"), 0, 0)
        layout.addWidget(self.camera_label, 1, 0, 1, 2)
        layout.addWidget(QLabel("Select Camera"), 2, 0)
        layout.addWidget(self.camera_selector, 2, 1)
        layout.addWidget(self.joystick, 3, 1, 1, 1, alignment=Qt.AlignRight)
        layout.addLayout(slider_layout, 3, 0, 1, 1, alignment=Qt.AlignLeft)

        self.setLayout(layout)

    def change_gear(self, value):
        self.velocity_control.set_gear(value)
        print(f"Changed to Gear: {value}")

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)

if __name__ == '__main__':
    rospy.init_node('rover_gui', anonymous=False)
    app = QApplication(sys.argv)
    gui = RoverGUI()
    gui.show()
    sys.exit(app.exec_())
