#!/usr/bin/env python3
#test instrcutions
# run saveSateliteTiles.py to generate test sample data 
#run fakeGPSPub.py to publish gps points
#play rosbag to view camera toggle

import sys
from enum import Enum
import rospy
import json
import numpy as np
import map_viewer as map_viewer
from pathlib import Path


from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame, QCheckBox,QSplitter,QSizePolicy
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix  # Import GPS message type
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter

CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"

class mapOverlay(QWidget):
    def __init__(self):
        super().__init__()
        
        self.viewer = map_viewer.MapViewer()
        self.viewer.set_map_server(
            str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
        )
        self.setLayout(self.initOverlayout())
        self.centreOnRover = False
        
        # ROS Subscriber for GPS coordinates
        rospy.Subscriber('/gps/fix', NavSatFix, self.update_gps_coordinates)
    
    def initOverlayout(self):
        OverLayout = QGridLayout()
        OverLayout.addWidget(self.viewer)
        return OverLayout

    def update_gps_coordinates(self, msg):
        gps_point = (msg.latitude, msg.longitude)
        self.viewer.set_robot_position(msg.latitude,msg.longitude)
        if self.centreOnRover == True:
            self.viewer.center_on_gps( gps_point)  # Trigger repaint



class Direction: 
    def __init__(self):
        self.LinX =0
        self.AngleZ = 0

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
        self.direction = Direction()
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
        #add more cameras here
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
        # self.setGeometry(100, 100, 1200, 800)
        self.setGeometry(50, 50, 100, 100)
        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Create tabs
        self.control_tab = QWidget()
        self.map_tab = QWidget()
        self.split_screen_tab = QWidget()


        # Add tabs to QTabWidget
        self.tabs.addTab(self.control_tab, "Controls")
        self.tabs.addTab(self.map_tab, "Map")
        self.tabs.addTab(self.split_screen_tab, "Split Screen")

        # self.tabs.addTab(self.splitScreenTab,"Split Screen")

        # Setup each tab
        self.setup_control_tab()
        self.setup_map_tab()
        self.setup_split_screen_tab()


    def setup_split_screen_tab(self):
        splitter = QSplitter(Qt.Horizontal)
        # Add camera feed to the splitter
        camera_group = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout()
        camera_label = QLabel(self.split_screen_tab)
        # camera_label.setFixedSize(640, 480)
        self.camera_label.setMinimumSize(320, 240)  # Allow it to shrink
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        camera_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        camera_layout.addWidget(camera_label)
        camera_group.setLayout(camera_layout)
         # Add map to the splitter
        map_group = QGroupBox("Map")
        map_layout = QVBoxLayout()
        self.map_overlay = mapOverlay()
        map_layout.addWidget(self.map_overlay)
        map_group.setLayout(map_layout)
        # Add widgets to the splitter
        splitter.addWidget(camera_group)
        splitter.addWidget(map_group)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        # Controls section
        controls_group = QGroupBox("Controls")
        controls_layout = QHBoxLayout()

        # Joystick
        self.joystick = Joystick(self.velocity_control)
        joystick_group = QGroupBox("Joystick")
        joystick_layout = QVBoxLayout()
        joystick_layout.addWidget(self.joystick)
        joystick_group.setLayout(joystick_layout)

        # Gear slider
        gear_group = QGroupBox("Gear Control")
        slider_layout = QVBoxLayout()
        self.gear_slider = QSlider(Qt.Horizontal, self.split_screen_tab)
        self.gear_slider.setRange(1, 10)
        self.gear_slider.setTickPosition(QSlider.TicksBelow)
        self.gear_slider.setTickInterval(1)
        self.gear_slider.valueChanged.connect(self.change_gear)
        slider_layout.addWidget(self.gear_slider)
        gear_group.setLayout(slider_layout)

        # Add joystick and gear controls side by side
        controls_layout.addWidget(joystick_group)
        controls_layout.addWidget(gear_group)
        controls_group.setLayout(controls_layout)

        # Layout for the split screen tab
        split_screen_layout = QVBoxLayout()
        split_screen_layout.addWidget(splitter)
        split_screen_layout.addWidget(controls_group)

        self.split_screen_tab.setLayout(split_screen_layout)
        
    def setup_control_tab(self):
        # Camera Group Box (top)
        camera_group = QGroupBox("Camera Feed")
        camera_group.setAlignment(Qt.AlignCenter)
        camera_layout = QVBoxLayout()
        
        # Camera feed widget with fixed size
        self.camera_label = QLabel(self.control_tab)
        self.camera_label.setMinimumSize(320, 240)  # Allow it to shrink
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)  # Add border style

        # Camera toggle with label
        camera_selector_layout = QHBoxLayout()
        self.camera_selector = QComboBox(self.control_tab)
        #add coresponding camera labels here
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
        joystick_group = QGroupBox("Joystick")
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

        label_layout.setSpacing(0)  # Decrease spacing between the labels
        slider_layout.addLayout(label_layout)
        slider_layout.addWidget(self.gear_slider)

        # Gear Group Box
        gear_group = QGroupBox("Gear Control")
        gear_group.setLayout(slider_layout)

        # Horizontal layout for joystick and gear control side by side
        control_layout = QHBoxLayout()
        control_layout.addWidget(joystick_group)  
        control_layout.addWidget(gear_group)  
        

        # Main Layout (Vertical: Camera on top, Joystick & Gear side by side below)
        main_layout = QVBoxLayout()
        main_layout.addWidget(camera_group)  
        main_layout.addLayout(control_layout)  

        # Set the final layout for the control tab
        self.control_tab.setLayout(main_layout)


    def setup_map_tab(self):

        # Initialize mapOverlay
        self.map_overlay = mapOverlay()
        # Create a checkbox for settings
        self.checkbox_setting = QCheckBox("Recentre map when rover offscreen")
        self.checkbox_setting.setChecked(False)  # Set the default state to unchecked
        self.checkbox_setting.stateChanged.connect(self.on_checkbox_state_changed)
        # Layout setup for map tab
        layout = QVBoxLayout()
        layout.addWidget(self.checkbox_setting)
        layout.addWidget(self.map_overlay)

        self.map_tab.setLayout(layout)
    def on_checkbox_state_changed(self, state):
        if state == Qt.Checked:
            self.map_overlay.centreOnRover = True
            # Perform actions for the checked state
        else:
            self.map_overlay.centreOnRover = False
            # Perform actions for the unchecked state

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
            font-size: 18px;
            border: 1px solid gray;
            margin-top: 10px;
        }
        QLabel {
            font-size: 12px;
        }
        QComboBox, QSlider {
            font-size: 18px;
        }
        QTabWidget::pane {
            border: 1px solid gray;
            background: #e6e6e6;
        }
    """)

    gui.show()
    sys.exit(app.exec_())
