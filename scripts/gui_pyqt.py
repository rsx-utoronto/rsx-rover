#!/usr/bin/env python3
#test instrcutions
# run saveSateliteTiles.py to generate test sample data 
#run fakeGPSPub.py to publish gps points
#play rosbag to view camera toggle

import sys
from enum import Enum
import rospy
import json
import pygame
import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix  # Import GPS message type
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter
widgetWidth = 400
widgetHeight = 300

class PygameOverlay(QWidget):
    def __init__(self, map_image_path, metadata_path):
        super().__init__()
        self.setMinimumSize(widgetWidth, widgetHeight)
        pygame.init()
        self.surface = pygame.Surface((widgetWidth, widgetHeight))
        self.map_image = pygame.image.load(map_image_path)
        self.metadata = self.load_metadata(metadata_path)

        # GPS bounds
        self.latitude_min = self.metadata['bounds']['southwest']['lat']
        self.latitude_max = self.metadata['bounds']['northeast']['lat']
        self.longitude_min = self.metadata['bounds']['southwest']['lng']
        self.longitude_max = self.metadata['bounds']['northeast']['lng']
        
        # History of GPS coordinates to draw the path
        self.gps_history = []
        
        # Zoom level and offset
        self.zoom_factor = 2.0  # Adjust to zoom in; 1.0 means no zoom
        self.offset_x = 0  # Horizontal offset to center the latest GPS coordinate
        self.offset_y = 0  # Vertical offset to center the latest GPS coordinate

        # ROS Subscriber for GPS coordinates
        rospy.Subscriber('/gps/fix', NavSatFix, self.update_gps_coordinates)

    def load_metadata(self, path):
        with open(path, 'r') as f:
            return json.load(f)

    def update_gps_coordinates(self, msg):
        gps_point = (msg.latitude, msg.longitude)
        
        # Append new GPS point to history
        self.gps_history.append(gps_point)
        
        # Limit history to avoid excessive memory usage
        if len(self.gps_history) > 1000:  # Keep last 1000 points (adjust as needed)
            self.gps_history.pop(0)
        
        self.update()  # Trigger repaint

    def center_on_gps(self, gps_point):
        """Set the initial offset to keep the center of the map fixed."""
        # Calculate the map's center (this will not change dynamically)

        # Adjust offset based on zoom level and widget dimensions
        image_width, image_height = self.map_image.get_size()
        self.offset_x = (image_width * self.zoom_factor - widgetWidth) // 2
        self.offset_y = (image_height * self.zoom_factor - widgetHeight) // 2


    def gps_to_pixel(self, latitude, longitude):
        x = int((longitude - self.longitude_min) / (self.longitude_max - self.longitude_min) * widgetWidth)
        y = int((1 - (latitude - self.latitude_min) / (self.latitude_max - self.latitude_min)) * widgetHeight)
        return x, y

    def paintEvent(self, event):
        self.render_map_and_gps()
        frame = pygame.surfarray.array3d(self.surface)
        frame = np.rot90(frame)
        frame = np.flipud(frame)
        height, width, _ = frame.shape
        img = QImage(frame.tobytes(), width, height, QImage.Format_RGB888)
        painter = QPainter(self)
        painter.drawImage(0, 0, img)

    def render_map_and_gps(self):
        # Clear surface
        self.surface.fill((255, 255, 255))

        # Calculate scaled dimensions that preserve aspect ratio
        image_width, image_height = self.map_image.get_size()
        aspect_ratio = image_width / image_height

        # Determine new width and height based on zoom, preserving aspect ratio
        if aspect_ratio > 1:  # Wider than tall
            map_width = int(image_width * self.zoom_factor)
            map_height = int(map_width / aspect_ratio)
        else:  # Taller than wide or square
            map_height = int(image_height * self.zoom_factor)
            map_width = int(map_height * aspect_ratio)

        zoomed_map = pygame.transform.scale(self.map_image, (map_width, map_height))
        
        # Blit the zoomed map with offset
        self.surface.blit(zoomed_map, (-self.offset_x, -self.offset_y))
        
        # Draw path of GPS points
        if len(self.gps_history) > 1:
            for i in range(1, len(self.gps_history)):
                start_x, start_y = self.gps_to_pixel(*self.gps_history[i - 1])
                end_x, end_y = self.gps_to_pixel(*self.gps_history[i])

                # Adjust coordinates based on zoom and offset
                start_x = int(start_x * self.zoom_factor - self.offset_x)
                start_y = int(start_y * self.zoom_factor - self.offset_y)
                end_x = int(end_x * self.zoom_factor - self.offset_x)
                end_y = int(end_y * self.zoom_factor - self.offset_y)

                pygame.draw.line(self.surface, (0, 0, 255), (start_x, start_y), (end_x, end_y), 2)
        
        # Draw a circle at the most recent GPS point
        if self.gps_history:
            recent_x, recent_y = self.gps_to_pixel(*self.gps_history[-1])
            recent_x = int(recent_x * self.zoom_factor - self.offset_x)
            recent_y = int(recent_y * self.zoom_factor - self.offset_y)
            pygame.draw.circle(self.surface, (0, 255, 0), (recent_x, recent_y), 5)


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
        self.setMinimumSize(200, 200)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 100
        self.velocity_control = velocity_control

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-40, -40, 80, 80).translated(self.movingOffset)
        return QRectF(-40, -40, 80, 80).translated(self._center())

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



        # Define a small constant increment for smooth acceleration
        increment = 0.05

        # Gradually adjust current linear velocity towards the target
        if abs(linX - self.direction.LinX) < increment:
            self.direction.LinX = linX  # Close enough to target, snap to it
        elif linX > self.direction.LinX:
            self.direction.LinX += increment  # Increase linearly
        else:
            self.direction.LinX -= increment  # Decrease linearly

        # Gradually adjust current angular velocity towards the target
        if abs(angleZ - self.direction.AngleZ) < increment:
            self.direction.AngleZ = angleZ  # Close enough to target, snap to it
        elif angleZ > self.direction.AngleZ:
            self.direction.AngleZ += increment  # Increase angular velocity
        else:
            self.direction.AngleZ -= increment  # Decrease angular velocity

        # Send the updated velocities to the rover
        self.velocity_control.send_velocity(self.direction.LinX, self.direction.AngleZ)

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


class RoverGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Create tabs
        self.control_tab = QWidget()
        self.map_tab = QWidget()

        # Add tabs to QTabWidget
        self.tabs.addTab(self.control_tab, "Controls")
        self.tabs.addTab(self.map_tab, "Map")

        # Setup each tab
        self.setup_control_tab()
        self.setup_map_tab()

    def setup_control_tab(self):
        # Camera Group Box (top)
        camera_group = QGroupBox("Camera Feed")
        camera_group.setAlignment(Qt.AlignCenter)
        camera_layout = QVBoxLayout()
        
        # Camera feed widget with fixed size
        self.camera_label = QLabel(self.control_tab)
        self.camera_label.setFixedSize(640, 480)  # Set to smaller size
        self.camera_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)  # Add border style

        # Camera toggle with label
        camera_selector_layout = QHBoxLayout()
        self.camera_selector = QComboBox(self.control_tab)
        self.camera_selector.addItem("Zed (front) camera")
        self.camera_selector.addItem("Butt camera")
        self.camera_selector.currentIndexChanged.connect(self.switch_camera)

        # Add the label and combo box to the horizontal layout
        camera_label = QLabel("Select Camera:")  # Label next to combo box
        camera_selector_layout.addWidget(camera_label, alignment=Qt.AlignRight)
        camera_selector_layout.addWidget(self.camera_selector)
        camera_selector_layout.setAlignment(Qt.AlignCenter)

        # ROS functionality
        self.camera_feed = CameraFeed(self.camera_label)
        self.velocity_control = VelocityControl()

        # Layout for camera feed
        camera_layout.addWidget(self.camera_label, alignment=Qt.AlignCenter)
        camera_layout.addLayout(camera_selector_layout)  # Add the layout with the label and combo box
        camera_group.setLayout(camera_layout)

        # Joystick Group Box (side-by-side with gear)
        joystick_group = QGroupBox("Joystick Control")
        joystick_layout = QVBoxLayout()

        # Joystick widget with velocity control
        self.joystick = Joystick(self.velocity_control)
        joystick_layout.addWidget(self.joystick)
        joystick_group.setLayout(joystick_layout)

        # Gear slider
        self.gear_slider = QSlider(Qt.Horizontal, self.control_tab)
        self.gear_slider.setRange(1, 10)
        self.gear_slider.setTickPosition(QSlider.TicksBelow)
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

        # Gear Group Box
        gear_group = QGroupBox("Gear Control")
        gear_group.setLayout(slider_layout)

        # Horizontal layout for joystick and gear control side by side
        control_layout = QHBoxLayout()
        control_layout.addWidget(gear_group)  # Gear control on the right
        control_layout.addWidget(joystick_group)  # Joystick control on the left

        # Main Layout (Vertical: Camera on top, Joystick & Gear side by side below)
        main_layout = QVBoxLayout()
        main_layout.addWidget(camera_group)  # Camera Feed on top
        main_layout.addLayout(control_layout)  # Joystick & Gear side by side

        # Set the final layout for the control tab
        self.control_tab.setLayout(main_layout)


    def setup_map_tab(self):
        # Path to map image and metadata
        map_image_path = "/home/kjyshen223/rover_ws/src/rsx-rover/scripts/map_output/map_screenshot.png"
        metadata_path = "/home/kjyshen223/rover_ws/src/rsx-rover/scripts/map_output/map_metadata.json"

        # Initialize PygameOverlay
        self.map_overlay = PygameOverlay(map_image_path, metadata_path)

        # Layout setup for map tab
        layout = QVBoxLayout()
        layout.addWidget(self.map_overlay)

        self.map_tab.setLayout(layout)

    def change_gear(self, value):
        self.velocity_control.set_gear(value)
        print(f"Changed to Gear: {value}")

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)


if __name__ == '__main__':
    rospy.init_node('rover_gui', anonymous=False)
    app = QApplication(sys.argv)
    gui = RoverGUI()

     # Apply a basic stylesheet for a modern look
    app.setStyleSheet("""
        QMainWindow {
            background-color: #f5f5f5;
        }
        QGroupBox {
            font-weight: bold;
            font-size: 14px;
            border: 1px solid gray;
            margin-top: 10px;
        }
        QLabel {
            font-size: 12px;
        }
        QComboBox, QSlider {
            font-size: 12px;
        }
        QTabWidget::pane {
            border: 1px solid gray;
            background: #e6e6e6;
        }
    """)

    gui.show()
    sys.exit(app.exec_())
