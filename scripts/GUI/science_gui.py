#!/usr/bin/env python3

#Add vertical part -- two section -- nest that with a horizontal part - qhbox layout horizontal - qv vertical - plotwidget add tabs/section
#

import csv
import sys
import rclpy
from rclpy.node import Node
import map_viewer as map_viewer
from pathlib import Path
import numpy as np
import os
import time


from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, \
    QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame, \
    QCheckBox, QSplitter, QStylePainter, QStyleOptionComboBox, QStyle, \
    QToolButton, QMenu, QLineEdit, QPushButton, QTextEdit, \
    QListWidget, QListWidgetItem, QStyleOptionSlider, QSizePolicy, QScrollArea, QDialog
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from geometry_msgs.msg import Twist
# from rover.arm.ros1.gripper import arm_serial_connector
from sensor_msgs.msg import NavSatFix, CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float32, String, Bool, Int32
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter,QPalette,QStandardItemModel, QTextCursor, QFont
from calian_gnss_ros2_msg.msg import GnssSignalStatus
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for headless saving

#cache folder of map tiles generated from tile_scraper.py
CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"
science_arduino_board_name = None #Glob variable for board name, SHOULD BE CHANGED

#map widget that has map viewer 
class mapOverlay(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node=node
        
        self.viewer = map_viewer.MapViewer()
        #sets the source of map tiles to local tile cache folder
        self.viewer.set_map_server(
            str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
        )
        self.setLayout(self.initOverlayout())
        self.centreOnRover = False
        
        # ROS Subscriber for GPS coordinates
        node.create_subscription( NavSatFix, '/calian_gnss/gps', self.update_gps_coordinates, 10)
        node.create_subscription( GnssSignalStatus, '/calian_gnss/gps_extended',self.update_gps_heading, 10)
    
    #initialize overall layout
    def initOverlayout(self):
        OverLayout = QGridLayout()
        OverLayout.addWidget(self.viewer)
        return OverLayout

    #will redraw robot position when gps point received
    def update_gps_coordinates(self, msg):
        gps_point = (msg.latitude, msg.longitude)
        self.viewer.set_robot_position(msg.latitude,msg.longitude)
        if self.centreOnRover == True:
            self.viewer.center_on_gps(gps_point) 

    def update_gps_heading(self, msg):
        self.viewer.headingSignal.emit(msg.heading)

    def clear_map(self):
        self.viewer.clear_lines()


#object type for direction of rover
class Direction: 
    def __init__(self):
        self.LinX =0
        self.AngleZ = 0

class statusTerminal(QWidget):
    update_status_signal = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.init_ui()

        # Connect signals to the corresponding update methods
        self.update_status_signal.connect(self.update_string_list)
        # rospy.Subscriber('gui_status', String, self.string_callback)
        self.received_strings = []
        self.strlength = -1
    def init_ui(self):
        
        # Create a scrollable box for received strings
        self.string_list = QTextEdit(self)
        self.string_list.setReadOnly(True)
        self.string_list.setStyleSheet("""
            background-color: #FFFFFF; 
            color: black; 
            border: 2px solid black;  
            padding: 5px; 
        """)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_text)
        self.clear_button.setStyleSheet("""
            background-color: #FF5252;
            color: white;
            border: 2px solid black;
            border-radius: 10px;
            padding: 10px;
        """)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.clear_button)
        layout.addWidget(self.string_list)
        self.setLayout(layout)

    def clear_text(self):
        self.string_list.clear()
        self.received_strings = []
        self.strlength = -1
        self.string_list.setPlainText("")
        self.string_list.moveCursor(QTextCursor.Start)
    
    def string_callback(self, msg):
        self.update_status_signal.emit(msg.data.strip())  

    def update_string_list(self, new_string):
        self.received_strings.append(new_string)
        cursor_pos = self.string_list.textCursor().position()
        # self.string_list.setPlainText("\n".join(self.received_strings))
        self.string_list.append(new_string)
        if cursor_pos < self.strlength - self.received_strings[-1].__len__():
            self.string_list.moveCursor(QTextCursor.End)
        self.strlength += len(new_string) + 1

class ArucoWidget(QWidget):
    # Define signals to communicate with the main thread
    update_label_signal = pyqtSignal(bool)
    update_list_signal = pyqtSignal(str)

    def __init__(self,node):
        super().__init__()
        self.init_ui()
        self.node=node

        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        node.create_subscription( Bool,'aruco_found', self.bool_callback, 10)
        node.create_subscription( String,'aruco_name', self.string_callback, 10)

        self.received_strings = []

    def init_ui(self):
        # Create a label
        self.label = QLabel("Aruco not found", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("""
            background-color: #808080; 
            color: white;  
            border: 2px solid black;  
            border-radius: 10px; 
            padding: 10px; 
        """)
        self.label.setFont(QFont("Arial", 72, QFont.Bold))

        # Create a scrollable box for received strings
        self.string_list = QTextEdit(self)
        self.string_list.setReadOnly(True)
        self.string_list.setStyleSheet("""
            background-color: #FFFFFF; 
            color: black; 
            border: 2px solid black;  
            padding: 5px; 
        """)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.string_list)
        self.setLayout(layout)

    def bool_callback(self, msg):
        # Emit signal to update the label in the main thread
        self.update_label_signal.emit(msg.data)

    def string_callback(self, msg):
        # Emit signal to update the list in the main thread
        self.update_list_signal.emit(msg.data.strip())

    def update_label(self, found):
        # Update the label in the main thread
        if found:
            self.label.setText("Aruco Found")
            self.label.setStyleSheet("""
                background-color: #4CAF50; 
                color: white;   
                border: 2px solid black; 
                border-radius: 10px;  
                padding: 10px; 
            """)
        else:
            self.label.setText("Aruco not found")
            self.label.setStyleSheet("""
                background-color: #FF5252; 
                color: white;  
                border: 2px solid black;  
                border-radius: 10px;  
                padding: 10px;  
            """)

    def update_string_list(self, new_string):
        # Append the string to the list in the main thread
        self.received_strings.append(new_string)
        self.string_list.setPlainText("\n".join(self.received_strings))
        self.string_list.moveCursor(QTextCursor.End)

class ArucoBar(QWidget):
    # Define signals to communicate with the main thread
    update_label_signal = pyqtSignal(bool)
    update_list_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.init_ui()
        self.node=node

        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        node.create_subscription(Bool, 'aruco_found', self.bool_callback, 10)
        node.create_subscription(String, 'aruco_name', self.string_callback, 10)
        
        self.received_string = ""

    def init_ui(self):
        # Create a label
        self.label = QLabel("Aruco not found", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("""
            background-color: #808080; 
            color: white;  
            border: 2px solid black;  
            border-radius: 10px; 
            padding: 10px; 
        """)
        self.label.setFont(QFont("Arial", 72, QFont.Bold))

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def bool_callback(self, msg):
        # Emit signal to update the label in the main thread
        self.update_label_signal.emit(msg.data)

    def string_callback(self, msg):
        # Emit signal to update the list in the main thread
        self.update_list_signal.emit(msg.data.strip())

    def update_label(self, found):
        # Update the label in the main thread
        if found:
            self.label.setText("Aruco Found: " + self.received_string)
            self.label.setStyleSheet("""
                background-color: #4CAF50; 
                color: white;   
                border: 2px solid black; 
                border-radius: 10px;  
                padding: 10px; 
            """)
        else:
            self.label.setText("Aruco not found")
            self.label.setStyleSheet("""
                background-color: #FF5252; 
                color: white;  
                border: 2px solid black;  
                border-radius: 10px;  
                padding: 10px;  
            """)

    def update_string_list(self, new_string):
        # Append the string to the list in the main thread
        self.received_string = new_string

class ObjectBar(QWidget):
    # Define signals to communicate with the main thread
    update_mallet_signal = pyqtSignal(bool)
    update_bottle_signal = pyqtSignal(bool)

    def __init__(self, node):
        super().__init__()
        self.init_ui()
        self.node = node
        # Connect signals to the corresponding update methods
        self.update_mallet_signal.connect(self.update_mallet)
        self.update_bottle_signal.connect(self.update_bottle)

        # Initialize ROS subscribers
        node.create_subscription(Bool, 'mallet_detected', self.mallet_callback, 10)
        node.create_subscription(Bool, 'waterbottle_detected', self.bottle_callback, 10)

        self.received_strings = []

    def init_ui(self):
        # Create a label
        self.label_mallet = QLabel("Mallet not found", self)
        self.label_mallet.setAlignment(Qt.AlignCenter)
        self.label_mallet.setStyleSheet("""
            background-color: #808080; 
            color: white;  
            border: 2px solid black;  
            border-radius: 10px; 
            padding: 10px; 
        """)
        self.label_mallet.setFont(QFont("Arial", 72, QFont.Bold))

        # Create a scrollable box for received strings
        self.label_bottle = QLabel("Waterbottle not found", self)
        self.label_bottle.setAlignment(Qt.AlignCenter)
        self.label_bottle.setStyleSheet("""
            background-color: #808080; 
            color: white;  
            border: 2px solid black;  
            border-radius: 10px; 
            padding: 10px; 
        """)
        self.label_bottle.setFont(QFont("Arial", 72, QFont.Bold))

        # Layout
        layout = QHBoxLayout()
        layout.addWidget(self.label_mallet)
        layout.addWidget(self.label_bottle)
        self.setLayout(layout)

    def mallet_callback(self, msg):
        # Emit signal to update the label in the main thread
        self.update_mallet_signal.emit(msg.data)

    def bottle_callback(self, msg):
        # Emit signal to update the list in the main thread
        self.update_bottle_signal.emit(msg.data)

    def update_mallet(self, found):
        # Update the label in the main thread
        if found:
            self.label_mallet.setText("Mallet Found")
            self.label_mallet.setStyleSheet("""
                background-color: #4CAF50; 
                color: white;   
                border: 2px solid black; 
                border-radius: 10px;  
                padding: 10px; 
            """)
        else:
            self.label_mallet.setText("Mallet not found")
            self.label_mallet.setStyleSheet("""
                background-color: #FF5252; 
                color: white;  
                border: 2px solid black;  
                border-radius: 10px;  
                padding: 10px;  
            """)

    def update_bottle(self, found):
        if found:
            self.label_bottle.setText("Waterbottle Found")
            self.label_bottle.setStyleSheet("""
                background-color: #4CAF50; 
                color: white;   
                border: 2px solid black; 
                border-radius: 10px;  
                padding: 10px; 
            """)
        else:
            self.label_bottle.setText("Waterbottle not found")
            self.label_bottle.setStyleSheet("""
                background-color: #FF5252; 
                color: white;  
                border: 2px solid black;  
                border-radius: 10px;  
                padding: 10px;  
            """)

class StateMachineStatus(QWidget):
    # Define signal to update the label
    update_label_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.init_ui()
        self.node=node
        # Connect the signal to the update method
        self.update_label_signal.connect(self.update_label)

        # Initialize ROS subscriber
        # rospy.Subscriber('/led_colour', String, self.callback)
        node.create_subscription(String, '/led_colour', self.callback, 10)

    def init_ui(self):
        # Create a label
        self.label = QLabel("Uninitialized LED", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("""
            background-color: #808080; 
            color: white;  
            border: 2px solid black; 
            border-radius: 10px; 
            padding: 10px; 
        """)
        self.label.setFont(QFont("Arial", 16, QFont.Bold))

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def callback(self, msg):
        # Emit signal to update the label in the main thread
        self.update_label_signal.emit(msg.data)

    def update_label(self, color):
        # Update the label based on the color in the main thread
        if color == "red":
            self.label.setText("Red status message")
            self.label.setStyleSheet("""
                background-color: red;  
                color: black;           
                border: 2px solid black;
                border-radius: 10px;
                padding: 10px;
            """)
        elif color == "green":
            self.label.setText("Green status message")
            self.label.setStyleSheet("""
                background-color: green;  
                color: black;            
                border: 2px solid black;
                border-radius: 10px;
                padding: 10px;
            """)
        elif color == "yellow":
            self.label.setText("Yellow status message")
            self.label.setStyleSheet("""
                background-color: yellow;  
                color: black;              
                border: 2px solid black;
                border-radius: 10px;
                padding: 10px;
            """)
        



#type bars widget for latitude longitude entry
class EditableComboBox(QComboBox):
    def __init__(self):
        super().__init__()
        

        self.setEditable(False)  # Using a QListWidget for custom items
        self.list_widget = QListWidget()
        self.setModel(self.list_widget.model())
        self.setView(self.list_widget)

        self.items_data = []  # Stores references to text edit fields
        self.populate_items()

    def populate_items(self):
        coordArray =["Start", "GNSS 1","GNSS 2", "AR 1", "AR 2", "AR 3", "OBJ 1", "OBJ 2"]
        for i in range(8):  # Example: 5 items in dropdown
            item_widget = QWidget()
            layout = QHBoxLayout()

            label = QLabel(f"Item {coordArray[i]}")  # Static text (not editable)
            text1 = QLineEdit()  # Editable box 1
            text2 = QLineEdit()  # Editable box 2

            self.items_data.append((label, text1, text2))  # Store references

            layout.addWidget(label)
            layout.addWidget(text1)
            layout.addWidget(text2)
            layout.setContentsMargins(0, 0, 0, 0)
            item_widget.setLayout(layout)

            item = QListWidgetItem(self.list_widget)
            item.setSizeHint(item_widget.sizeHint())
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, item_widget)

    def get_all_data(self):
        data = []
        for label, text1, text2 in self.items_data:
            if text1.text()=="":
                data.append(0)
            else:
                data.append(float(text1.text()))
            if text2.text()=="":
                data.append(0)
            else:
                data.append( float(text2.text()))
        return data

class LngLatEntryBar(QWidget):
    def __init__(self,node, map_overlay):
        super().__init__()
        self.longLat_pub = node.create_publisher(Float32MultiArray, '/long_lat_goal_array', 5)
        self.array = Float32MultiArray()
        layout = QVBoxLayout(self)

        self.combo = EditableComboBox()
        layout.addWidget(self.combo)

        # Add button to collect data
        self.submit_button = QPushButton("Get Data")
        self.submit_button.clicked.connect(self.collect_data)
        self.submit_button.clicked.connect(self.plot_points)
        layout.addWidget(self.submit_button)

        self.setLayout(layout)
        self.viewer = map_overlay

        self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')

    def collect_data(self):
        data = self.combo.get_all_data()
        print("Collected Data:")
        self.array.data = data
        self.longLat_pub.publish(self.array)
        for item in data:
            print(item)  # Prints each row's values

    def plot_points(self):
        data = self.combo.get_all_data()
        print("Collected Data:")
        
        # Create MapPoint objects from the data (pairs of lat, lng values)
        for i in range(0, len(data), 2):
            if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
                lat = data[i]
                lng = data[i+1]
                # Create a MapPoint with a radius of 5 and a name based on index
                point_name = f"Point {i//2 + 1}"
                print(f"{point_name}: {lat}, {lng}")
                self.viewer.viewer.goal_points[i//2].setLatLng([lat, lng])

        

class LngLatEntryFromFile(QWidget):
    def __init__(self, node, map_overlay):
        super().__init__()
        self.node=node
        self.longLat_pub = node.create_publisher(Float32MultiArray, '/long_lat_goal_array', 5)
        self.array = Float32MultiArray()
        layout = QVBoxLayout(self)

        self.submit_button = QPushButton("Get Data From File")
        self.submit_button.clicked.connect(self.collect_data)
        self.submit_button.clicked.connect(self.plot_points)
        layout.addWidget(self.submit_button)
        self.setLayout(layout)

        self.viewer = map_overlay

        self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')

    def collect_data(self):
        # Read data from the file
        file_path = Path(__file__).parent.parent.parent.resolve() / "long_lat_goal.csv"
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Process each line and publish
        data = []
        for line in lines:
            t = list(map(float, line.strip().split(',')[1:3]))
            for i in t:
                data.append(i)
        self.array.data = data
        self.longLat_pub.publish(self.array)
        print("Published Data:", data)

    def plot_points(self):
        data = self.array.data
        print("Plotting points from file data")
        
        # Create MapPoint objects from the data (pairs of lat, lng values)
        # map_points = []
        for i in range(0, len(data), 2):
            if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
                lat = data[i]
                lng = data[i+1]
                # Create a MapPoint with a radius of 5 and a name based on index
                point_name = f"Point {i//2 + 1}"
                # map_points.append((lat, lng, 5, point_name))
                print(f"{point_name}: {lat}, {lng}")
                self.viewer.viewer.goal_points[i//2].setLatLng([lat, lng])


class LngLatDeliveryEntryFromFile(QWidget):
    def __init__(self, map_overlay):
        super().__init__()
        self.array = Float32MultiArray()
        layout = QVBoxLayout(self)

        self.submit_button = QPushButton("Get Delivery Data From File")
        self.submit_button.clicked.connect(self.collect_data)
        layout.addWidget(self.submit_button)
        self.setLayout(layout)

        self.viewer = map_overlay

        self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')

    def collect_data(self):
        # Read data from the file
        file_path = Path(__file__).parent.parent.parent.resolve() / "delivery_lat_lon_goal.csv"
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Process each line and publish
        data = []
        for line in lines:
            t = list(map(float, line.strip().split(',')[1:3]))
            n = line.strip().split(',')[0]
            data.append(n)
            for i in t:
                data.append(i)
        print("Plotting points from file data")
        print(data)
        
        # Create MapPoint objects from the data (pairs of lat, lng values)
        # map_points = []
        for i in range(0, len(data), 3):
            if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
                lat = data[i+1]
                lng = data[i+2]
                # Create a MapPoint with a radius of 5 and a name based on index
                point_name = data[i]
                # map_points.append((lat, lng, 5, point_name))
                print(f"{point_name}: {lat}, {lng}")
                self.viewer.viewer.add_goal(point_name, lat, lng)
           

class VelocityControl:
    def __init__(self, node):
        self.node=node
        self.pub = node.create_publisher( Twist, '/drive',10)
        self.gear = 1

    def set_gear(self, gear):
        self.gear = gear
        print(f"Gear set to: {gear}")

    #publishes velocity to /drive topic
    def send_velocity(self, linear_x, angular_z):
        linear_x *= (self.gear / 10) * 2.5
        angular_z *= (self.gear / 10) * 2.5
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)
        print(f"Publishing to /drive: linear_x = {linear_x}\n, angular_z = {angular_z}\n, gear = {self.gear}")

#joystick that controlss velocity magnitude and direction
class Joystick(QWidget):
    joystickMoved = pyqtSignal(float, float)
    def __init__(self, velocity_control, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(200, 200)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 100
        self.direction = Direction()
        self.velocity_control = velocity_control
        self.current_linX = 0
        self.current_angleZ = 0 
        self.target_linX = 0
        self.target_angleZ = 0
        self.smooth_timer = QTimer(self)  
        self.smooth_timer.timeout.connect(self.update_velocity)
        self.smooth_timer.start(100)  # 50ms update interval


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
            return

        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        target_linX = distance if 0 <= angle < 180 else -distance
        target_angleZ = 0

        if 0 <= angle < 90:
            target_angleZ = -(90 - angle) / 90
        elif 270 <= angle < 360:
            target_angleZ = -(angle - 270) / 90
        elif 90 <= angle < 180:
            target_angleZ = (angle - 90) / 90
        else:
            target_angleZ = (270 - angle ) / 90

        # Store target values (will be gradually reached in `update_velocity`)
        self.target_linX = target_linX
        self.target_angleZ = target_angleZ


    def update_velocity(self):
        """Gradually adjust current velocity towards the target values"""
        step = 0.1  # Acceleration step per tick

        # Update linear velocity smoothly
        if abs(self.target_linX - self.current_linX) < step:
            self.current_linX = self.target_linX  # Snap to target if close
        elif self.target_linX > self.current_linX:
            self.current_linX += step
        else:
            self.current_linX -= step

        # Update angular velocity smoothly
        if abs(self.target_angleZ - self.current_angleZ) < step:
            self.current_angleZ = self.target_angleZ  # Snap to target if close
        elif self.target_angleZ > self.current_angleZ:
            self.current_angleZ += step
        else:
            self.current_angleZ -= step

        # Send smooth velocity to ROS
        if not(self.current_linX == 0 and self.current_angleZ ==0):
            self.velocity_control.send_velocity(self.current_linX, self.current_angleZ)

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.velocity_control.send_velocity(0, 0)
        self.target_linX = 0
        self.target_angleZ = 0
        self.current_angleZ = 0
        self.current_linX = 0
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        self.joystickDirection()
        self.joystickMoved.emit(self.movingOffset.x(), self.movingOffset.y())
    def setJoystickPosition(self, x, y):
        """Update joystick position when receiving sync signal"""
        self.movingOffset = QPointF(x, y)
        self.update()



class Slider(QSlider):
    valueUpdated =pyqtSignal(int)  # Custom signal

    def __init__(self, orientation, parent=None):
        super(Slider, self).__init__(orientation, parent)
        self.setTickInterval(10)  # Set tick interval (adjust as needed)
        self.setTickPosition(QSlider.TicksBelow)  # Show ticks below (for horizontal)
        self.setSingleStep(10)  # Ensure movement in fixed steps
        self.valueChanged.connect(self.on_value_changed)  # Connect slider movement

    def mousePressEvent(self, event):
        super(Slider, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            val = self.pixelPosToRangeValue(event.pos())
            rounded_val = round(val / 10) * 10  # Snap to nearest tick (adjust step size)
            self.setValue(rounded_val)
            self.valueUpdated.emit(rounded_val)  # Emit updated value

    def pixelPosToRangeValue(self, pos):
        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        gr = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self)
        sr = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self)

        if self.orientation() == Qt.Horizontal:
            sliderLength = sr.width()
            sliderMin = gr.x()
            sliderMax = gr.right() - sliderLength + 1
        else:
            sliderLength = sr.height()
            sliderMin = gr.y()
            sliderMax = gr.bottom() - sliderLength + 1
        pr = pos - sr.center() + sr.topLeft()
        p = pr.x() if self.orientation() == Qt.Horizontal else pr.y()
        return QStyle.sliderValueFromPosition(self.minimum(), self.maximum(), p - sliderMin,
                                                        sliderMax - sliderMin, opt.upsideDown)

    def on_value_changed(self, value):
        rounded_val = round(value / 10) * 10  # Snap to nearest tick
        self.setValue(rounded_val)  # Force snapping
        self.valueUpdated.emit(rounded_val)  # Emit updated value


#camera feed that displays one camera at a time (use switch_camera)
# Update the CameraFeed class to handle multiple labels
class ResizableLabel(QLabel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setScaledContents(False)  # Avoid QLabel forcing image scaling

    def resizeEvent(self, event):
        if self.pixmap():
            self.setPixmap(self.pixmap().scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        super().resizeEvent(event)


class PixelSelectableLabel(QLabel):
    pixelSelected = pyqtSignal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setBackgroundRole(QPalette.Base)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._base_pixmap = None
        self._zoom_factor = 1.0
        self._image_width = 0
        self._image_height = 0

    def set_image_array(self, rgb_image: np.ndarray):
        if rgb_image is None:
            return
        self._image_height, self._image_width = rgb_image.shape[:2]
        bytes_per_line = 3 * self._image_width
        qimg = QImage(
            rgb_image.data,
            self._image_width,
            self._image_height,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()
        self._base_pixmap = QPixmap.fromImage(qimg)
        self._render_pixmap()

    def set_zoom(self, zoom_factor: float):
        self._zoom_factor = max(0.1, zoom_factor)
        self._render_pixmap()

    def _render_pixmap(self):
        if self._base_pixmap is None:
            return
        target_width = max(1, int(self._base_pixmap.width() * self._zoom_factor))
        target_height = max(1, int(self._base_pixmap.height() * self._zoom_factor))
        scaled = self._base_pixmap.scaled(
            target_width,
            target_height,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.setPixmap(scaled)
        self.resize(scaled.size())

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.pixmap() and self._image_width and self._image_height:
            pix = self.pixmap()
            x_display = event.pos().x()
            y_display = event.pos().y()
            if 0 <= x_display < pix.width() and 0 <= y_display < pix.height():
                x_image = int((x_display / pix.width()) * self._image_width)
                y_image = int((y_display / pix.height()) * self._image_height)
                x_image = max(0, min(self._image_width - 1, x_image))
                y_image = max(0, min(self._image_height - 1, y_image))
                self.pixelSelected.emit(x_image, y_image)
        super().mousePressEvent(event)


class CameraFeed:
    def __init__(self, node, label1, label2, label3, label4, label5, splitter):
        self.node=node
        self.last_genie_image = None
        self.active_cameras = {"Zed (front) camera": False, "Butt camera": False, "Microscope camera": False, "Genie camera": False, "Webcam": False}
        self.bridge = CvBridge()
        self.image_sub1 = None
        self.image_sub2 = None
        self.image_sub3 = None
        self.image_sub4 = None
        self.image_sub5 = None
        self.state_sub = node.create_subscription(String, "state", self.state_callback,10)

        self.obj_bbox = node.create_subscription( Float64MultiArray,"object/bbox", self.bbox_callback, 10)
        self.bbox_sub = node.create_subscription( Float64MultiArray,"aruco_node/bbox", self.bbox_callback, 10)

        self.label1 = label1
        self.label2 = label2
        self.label3 = label3
        self.label4 = label4
        self.label5 = label5
        self.splitter = splitter

        # Set size policies correctly
        self.label1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label4.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label5.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label1.setMinimumSize(300, 200)
        self.label2.setMinimumSize(300, 200)
        self.label3.setMinimumSize(300, 200)
        self.label4.setMinimumSize(300, 200)
        self.label5.setMinimumSize(300, 200)
        self.splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.bbox = None  

        self.bbox_timer = QTimer()
        self.bbox_timer.setInterval(1000)
        self.bbox_timer.timeout.connect(self.clear_bbox)
        self.bbox_timer.setSingleShot(True)

        self.exposure_value = 50

        self.label1.hide()
        self.label2.hide()
        self.label3.hide()
        self.label4.hide()
        self.label5.hide()

    def register_subscriber1(self):
        if self.image_sub1 is None:
            self.image_sub1 = node.create_subscription(CompressedImage,"/zed_node/rgb/image_rect_color/compressed",  self.callback1, 10)

    def unregister_subscriber1(self):
        if self.image_sub1:
            self.image_sub1.unregister()
            self.image_sub1 = None

    def register_subscriber2(self):
        if self.image_sub2 is None:
            self.image_sub2 = node.create_subscription( CompressedImage, "/camera2/camera/color/image_raw/compressed", self.callback2, 10)

    def unregister_subscriber2(self):
        if self.image_sub2:
            self.image_sub2.unregister()
            self.image_sub2 = None

    def register_subscriber3(self):
        if self.image_sub3 is None:
            self.image_sub3 = node.create_subscription(CompressedImage, "/microscope/compressed", self.callback3, 10)

    def unregister_subscriber3(self):
        if self.image_sub3:
            self.image_sub3.unregister()
            self.image_sub3 = None
        
    def register_subscriber4(self):
        if self.image_sub4 is None:
            self.image_sub4 = node.create_subscription( Image,"/geniecam", self.callback4, 10)
    
    def unregister_subscriber4(self):
        if self.image_sub4:
            self.image_sub4.unregister()
            self.image_sub4 = None

    def register_subscriber5(self):
        if self.image_sub5 is None:
            self.image_sub5 = node.create_subscription( CompressedImage, "/webcam/compressed",self.callback5, 10)

    def unregister_subscriber5(self):
        if self.image_sub5:
            self.image_sub5.unregister()
            self.image_sub5 = None

    def state_callback(self, msg):
        self.state = msg.data

    def bbox_callback(self, msg):
        if len(msg.data) == 8:
            self.bbox = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[5])]
        else:
            self.bbox = None  

        QMetaObject.invokeMethod(self.bbox_timer, "start", Qt.QueuedConnection)

    def clear_bbox(self):
        self.bbox = None
        self.bbox_timer.stop()
        QMetaObject.invokeMethod(self.label1, "update", Qt.QueuedConnection)

    def callback1(self, data):
        if self.active_cameras["Zed (front) camera"]:
            self.update_image(data, self.label1)

    def callback2(self, data):
        if self.active_cameras["Butt camera"]:
            self.update_image(data, self.label2)
    
    def callback3(self, data):
        # print("Microscope camera callback")
        if self.active_cameras["Microscope camera"]:
            # print("get image")
            self.update_image(data, self.label3)

    def callback4(self, data):
        if self.active_cameras["Genie camera"]:
            self.last_genie_image = data
            self.update_genie_image(data, self.label4)

    def callback5(self, data):
        if self.active_cameras["Webcam"]:
            self.update_image(data, self.label5)

    def update_image(self, data, label):
        """Decode and update the camera image with bounding box."""
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            return  

        alpha = self.exposure_value / 50.0   # contrast: 0.02 to 2.0
        beta = (self.exposure_value - 50) * 2  # brightness: -98 to +100

        cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        if label == self.label1 and self.bbox:
            x1, y1, x2, y2 = self.bbox
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

        height, width, _ = cv_image.shape
        bytes_per_line = 3 * width
        qimg = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        # Ensure the pixmap is resized before setting it to QLabel
        scaled_pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)

        def update_label():
            if label.pixmap() and label.pixmap().size() == scaled_pixmap.size():
                return  # Avoid unnecessary updates
            label.setPixmap(scaled_pixmap)

        QMetaObject.invokeMethod(label, "setPixmap", Qt.QueuedConnection, Q_ARG(QPixmap, scaled_pixmap))

    def update_microscope_image(self, data, label):
        """Decode and update a microscope camera ROS Image message."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data)
            if cv_image is None:
                print("Failed to convert microscope image from topic")
                return
                
            # Check the image shape to determine format
            if len(cv_image.shape) == 2:
                # It's grayscale (single channel)
                # Apply exposure/brightness adjustments
                alpha = self.exposure_value / 50.0   # contrast: 0.02 to 2.0
                beta = (self.exposure_value - 50) * 2  # brightness: -98 to +100
                cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
                
                # Convert grayscale to BGR for display
                cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                # It's already a color image
                # Apply exposure/brightness adjustments
                alpha = self.exposure_value / 50.0
                beta = (self.exposure_value - 50) * 2
                cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
                
                # Ensure it's in BGR format for OpenCV
                cv_image_rgb = cv_image
            
            # Convert to QImage - always use RGB format for Qt
            height, width = cv_image_rgb.shape[:2]
            bytes_per_line = 3 * width
            cv_image_rgb = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2RGB)
            qimg = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            
            # Scale pixmap to fit label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # Update label with new pixmap
            QMetaObject.invokeMethod(label, "setPixmap", Qt.QueuedConnection, Q_ARG(QPixmap, scaled_pixmap))
            
        except Exception as e:
            print(f"Error updating microscope image: {e}")

    def update_genie_image(self, data, label):
        """Decode and update a grayscale ROS Image message."""
        try:
            # Let CvBridge detect the encoding automatically instead of forcing mono8
            cv_image = self.bridge.imgmsg_to_cv2(data)
            if cv_image is None:
                print("Failed to convert image from topic")
                return
            
            # Check the image shape to determine format
            if len(cv_image.shape) == 2:
                # It's already grayscale (single channel)
                # Apply exposure/brightness adjustments
                alpha = self.exposure_value / 50.0   # contrast: 0.02 to 2.0
                beta = (self.exposure_value - 50) * 2  # brightness: -98 to +100
                cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
                
                # Convert grayscale to BGR for display
                cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                # It's already a color image
                # Apply exposure/brightness adjustments
                alpha = self.exposure_value / 50.0
                beta = (self.exposure_value - 50) * 2
                cv_image = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)
                
                # Ensure it's in BGR format for OpenCV
                cv_image_rgb = cv_image
            
            # Convert to QImage - always use RGB format for Qt
            height, width = cv_image_rgb.shape[:2]
            bytes_per_line = 3 * width
            cv_image_rgb = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2RGB)
            qimg = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            
            # Scale pixmap to fit label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # Update label with new pixmap
            QMetaObject.invokeMethod(label, "setPixmap", Qt.QueuedConnection, Q_ARG(QPixmap, scaled_pixmap))
            
        except Exception as e:
            print(f"Error updating genie image: {e}")

    def update_active_cameras(self, active_cameras):
        self.active_cameras = active_cameras
        self.update_subscribers()
        self.update_visibility()

    def update_subscribers(self):
        if self.active_cameras["Zed (front) camera"]:
            self.register_subscriber1()
        else:
            self.unregister_subscriber1()

        if self.active_cameras["Butt camera"]:
            self.register_subscriber2()
        else:
            self.unregister_subscriber2()

        if self.active_cameras["Microscope camera"]:
            self.register_subscriber3()
        else:
            self.unregister_subscriber3()

        if self.active_cameras["Genie camera"]:
            self.register_subscriber4()
        else:
            self.unregister_subscriber4()

        if self.active_cameras["Webcam"]:
            self.register_subscriber5()
        else:
            self.unregister_subscriber5()

    def update_visibility(self):
        active_count = sum(self.active_cameras.values())
        if self.active_cameras["Zed (front) camera"]:
            self.label1.show()
            self.splitter.setStretchFactor(0, 1)
        else:
            self.label1.hide()
            self.splitter.setStretchFactor(0, 0)
        if self.active_cameras["Butt camera"]:
            self.label2.show()
            self.splitter.setStretchFactor(1, 1)
        else:
            self.label2.hide()
            self.splitter.setStretchFactor(1, 0)
        if self.active_cameras["Microscope camera"]:
            self.label3.show()
            self.splitter.setStretchFactor(2, 1)
        else:
            self.label3.hide()
            self.splitter.setStretchFactor(2, 0)
        if self.active_cameras["Genie camera"]:
            self.label4.show()
            self.splitter.setStretchFactor(3, 1)
        else:
            self.label4.hide()
            self.splitter.setStretchFactor(3, 0)
        if self.active_cameras["Webcam"]:
            self.label5.show()
            self.splitter.setStretchFactor(4, 1)
        else:
            self.label5.hide()
            self.splitter.setStretchFactor(4, 0)

class CamerasWindow(QDialog):
    ultrasonicUpdateSignal = pyqtSignal(str)
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.bridge = CvBridge()

        self.setWindowTitle("Cameras Window")
        self.setGeometry(200, 200, 1200, 700)

        # -----------------------------
        # ROS2 topic placeholders
        # Replace these strings later with the actual topic names
        # -----------------------------
        self.camera1_topic = "/camera_1/image_raw/compressed"   # TODO: replace with real topic
        self.camera2_topic = "/camera_2/image_raw/compressed"   # TODO: replace with real topic
        self.camera3_topic = "/camera_3/image_raw/compressed"   # TODO: replace with real topic

        # -----------------------------
        # Electromagnet + ultrasonic placeholders
        # Replace these strings later with the actual topic names
        # -----------------------------
        self.electromagnet_topic = "/electromagnet_control"   # TODO: replace with real topic
        self.ultrasonic_topic = "/ultrasonic_sensor"          # TODO: replace with real topic

        self.electromagnet_on = False

        self.electromagnet_pub = self.node.create_publisher(
            Bool,
            self.electromagnet_topic,
            10
        )

        self.ultrasonic_sub = self.node.create_subscription(
            Float32,
            self.ultrasonic_topic,
            self.ultrasonic_callback,
            10
        )

        self.ultrasonicUpdateSignal.connect(self.update_ultrasonic_display)

        # If one or more cameras publish sensor_msgs/msg/Image
        # instead of sensor_msgs/msg/CompressedImage, set these to False.
        self.camera1_is_compressed = True
        self.camera2_is_compressed = True
        self.camera3_is_compressed = True

        # ROS2 subscriptions start OFF
        self.camera1_sub = None
        self.camera2_sub = None
        self.camera3_sub = None

        # -----------------------------
        # UI
        # -----------------------------
        main_layout = QVBoxLayout()

        title_label = QLabel("Cameras Window")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 20px; font-weight: bold;")

        feeds_layout = QHBoxLayout()

        # Camera 1 column
        cam1_layout = QVBoxLayout()
        self.camera_label1 = ResizableLabel("Camera 1")
        self.camera_label1.setAlignment(Qt.AlignCenter)
        self.camera_label1.setMinimumSize(320, 240)
        self.camera_label1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label1.setStyleSheet("background-color: black; color: white;")

        self.camera1_button = QPushButton("Turn Camera 1 On")
        self.camera1_button.setCheckable(True)
        self.camera1_button.clicked.connect(self.toggle_camera1)

        cam1_layout.addWidget(self.camera_label1)
        cam1_layout.addWidget(self.camera1_button)

        # Camera 2 column
        cam2_layout = QVBoxLayout()
        self.camera_label2 = ResizableLabel("Camera 2")
        self.camera_label2.setAlignment(Qt.AlignCenter)
        self.camera_label2.setMinimumSize(320, 240)
        self.camera_label2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label2.setStyleSheet("background-color: black; color: white;")

        self.camera2_button = QPushButton("Turn Camera 2 On")
        self.camera2_button.setCheckable(True)
        self.camera2_button.clicked.connect(self.toggle_camera2)

        cam2_layout.addWidget(self.camera_label2)
        cam2_layout.addWidget(self.camera2_button)

        # Camera 3 column
        cam3_layout = QVBoxLayout()
        self.camera_label3 = ResizableLabel("Camera 3")
        self.camera_label3.setAlignment(Qt.AlignCenter)
        self.camera_label3.setMinimumSize(320, 240)
        self.camera_label3.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label3.setStyleSheet("background-color: black; color: white;")

        self.camera3_button = QPushButton("Turn Camera 3 On")
        self.camera3_button.setCheckable(True)
        self.camera3_button.clicked.connect(self.toggle_camera3)

        cam3_layout.addWidget(self.camera_label3)
        cam3_layout.addWidget(self.camera3_button)

        feeds_layout.addLayout(cam1_layout)
        feeds_layout.addLayout(cam2_layout)
        feeds_layout.addLayout(cam3_layout)

        self.topic_info_label = QLabel(
            f"Camera 1: {self.camera1_topic}\n"
            f"Camera 2: {self.camera2_topic}\n"
            f"Camera 3: {self.camera3_topic}"
        )
        self.topic_info_label.setAlignment(Qt.AlignLeft)
        self.topic_info_label.setStyleSheet("padding: 8px;")

        # -----------------------------
        # Electromagnet + ultrasonic controls
        # -----------------------------
        extra_controls_group = QGroupBox("Pickup / Sensor Controls")
        extra_controls_layout = QHBoxLayout()

        self.electromagnet_button = QPushButton("Electromagnet Off")
        self.electromagnet_button.setCheckable(True)
        self.electromagnet_button.clicked.connect(self.toggle_electromagnet)
        self.electromagnet_button.setStyleSheet("""
            QPushButton:checked {
                background-color: orange;
                color: black;
                font-weight: bold;
            }
        """)

        self.ultrasonic_display_label = QLabel("Ultrasonic: --")
        self.ultrasonic_display_label.setAlignment(Qt.AlignCenter)
        self.ultrasonic_display_label.setMinimumHeight(40)
        self.ultrasonic_display_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.ultrasonic_display_label.setStyleSheet("""
            background-color: white;
            color: black;
            font-size: 18px;
            padding: 6px;
        """)

        extra_controls_layout.addWidget(self.electromagnet_button)
        extra_controls_layout.addWidget(self.ultrasonic_display_label)
        extra_controls_group.setLayout(extra_controls_layout)

        main_layout.addWidget(title_label)
        main_layout.addLayout(feeds_layout)
        main_layout.addWidget(extra_controls_group)
        main_layout.addWidget(self.topic_info_label)
        self.setLayout(main_layout)

    # -----------------------------
    # Toggle functions
    # -----------------------------
    def toggle_camera1(self):
        if self.camera1_button.isChecked():
            self.start_camera1()
        else:
            self.stop_camera1()

    def toggle_camera2(self):
        if self.camera2_button.isChecked():
            self.start_camera2()
        else:
            self.stop_camera2()

    def toggle_camera3(self):
        if self.camera3_button.isChecked():
            self.start_camera3()
        else:
            self.stop_camera3()

    def toggle_electromagnet(self):
        self.electromagnet_on = self.electromagnet_button.isChecked()

        msg = Bool()
        msg.data = self.electromagnet_on
        self.electromagnet_pub.publish(msg)

        if self.electromagnet_on:
            self.electromagnet_button.setText("Electromagnet On")
            print(f"Electromagnet ON published to {self.electromagnet_topic}")
        else:
            self.electromagnet_button.setText("Electromagnet Off")
            print(f"Electromagnet OFF published to {self.electromagnet_topic}")


    def ultrasonic_callback(self, msg):
        # Assumes std_msgs/msg/Float32
        # If your topic uses another message type, adjust this line.
        display_text = f"Ultrasonic: {msg.data:.2f}"
        self.ultrasonicUpdateSignal.emit(display_text)


    def update_ultrasonic_display(self, text):
        self.ultrasonic_display_label.setText(text)

    # -----------------------------
    # Start subscribers
    # -----------------------------
    def start_camera1(self):
        if self.camera1_sub is not None:
            return

        msg_type = CompressedImage if self.camera1_is_compressed else Image
        self.camera1_sub = self.node.create_subscription(
            msg_type,
            self.camera1_topic,
            self.camera1_callback,
            10
        )

        self.camera1_button.setText("Turn Camera 1 Off")
        print(f"Camera 1 started: {self.camera1_topic}")

    def start_camera2(self):
        if self.camera2_sub is not None:
            return

        msg_type = CompressedImage if self.camera2_is_compressed else Image
        self.camera2_sub = self.node.create_subscription(
            msg_type,
            self.camera2_topic,
            self.camera2_callback,
            10
        )

        self.camera2_button.setText("Turn Camera 2 Off")
        print(f"Camera 2 started: {self.camera2_topic}")

    def start_camera3(self):
        if self.camera3_sub is not None:
            return

        msg_type = CompressedImage if self.camera3_is_compressed else Image
        self.camera3_sub = self.node.create_subscription(
            msg_type,
            self.camera3_topic,
            self.camera3_callback,
            10
        )

        self.camera3_button.setText("Turn Camera 3 Off")
        print(f"Camera 3 started: {self.camera3_topic}")

    # -----------------------------
    # Stop subscribers
    # -----------------------------
    def stop_camera1(self):
        if self.camera1_sub is not None:
            self.node.destroy_subscription(self.camera1_sub)
            self.camera1_sub = None

        self.camera_label1.clear()
        self.camera_label1.setText("Camera 1")
        self.camera_label1.setStyleSheet("background-color: black; color: white;")
        self.camera1_button.setChecked(False)
        self.camera1_button.setText("Turn Camera 1 On")
        print("Camera 1 stopped")

    def stop_camera2(self):
        if self.camera2_sub is not None:
            self.node.destroy_subscription(self.camera2_sub)
            self.camera2_sub = None

        self.camera_label2.clear()
        self.camera_label2.setText("Camera 2")
        self.camera_label2.setStyleSheet("background-color: black; color: white;")
        self.camera2_button.setChecked(False)
        self.camera2_button.setText("Turn Camera 2 On")
        print("Camera 2 stopped")

    def stop_camera3(self):
        if self.camera3_sub is not None:
            self.node.destroy_subscription(self.camera3_sub)
            self.camera3_sub = None

        self.camera_label3.clear()
        self.camera_label3.setText("Camera 3")
        self.camera_label3.setStyleSheet("background-color: black; color: white;")
        self.camera3_button.setChecked(False)
        self.camera3_button.setText("Turn Camera 3 On")
        print("Camera 3 stopped")

    # -----------------------------
    # Camera callbacks
    # -----------------------------
    def camera1_callback(self, msg):
        self.update_camera_label(msg, self.camera_label1, self.camera1_is_compressed)

    def camera2_callback(self, msg):
        self.update_camera_label(msg, self.camera_label2, self.camera2_is_compressed)

    def camera3_callback(self, msg):
        self.update_camera_label(msg, self.camera_label3, self.camera3_is_compressed)

    def update_camera_label(self, msg, label, is_compressed):
        try:
            if is_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if cv_image is None:
                return

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            height, width, channel = cv_image.shape
            bytes_per_line = channel * width

            qimg = QImage(
                cv_image.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888
            )

            pixmap = QPixmap.fromImage(qimg)

            scaled_pixmap = pixmap.scaled(
                label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            label.setPixmap(scaled_pixmap)

        except Exception as e:
            print(f"Error updating camera feed: {e}")

    def closeEvent(self, event):
        self.stop_camera1()
        self.stop_camera2()
        self.stop_camera3()

        # Safety: turn electromagnet off when the camera window closes
        msg = Bool()
        msg.data = False
        self.electromagnet_pub.publish(msg)

        self.electromagnet_on = False
        self.electromagnet_button.setChecked(False)
        self.electromagnet_button.setText("Electromagnet Off")

        event.accept()

#main gui class, make updates here to change top level hierarchy
class RoverGUI(QMainWindow):
    statusSignal = pyqtSignal(str)
    probeUpdateSignal = pyqtSignal(bool)
    multispectralImageSavedSignal = pyqtSignal(dict)
    multispectralStatusSignal = pyqtSignal(str)
    def __init__(self, node:Node):
        self.node=node
        super().__init__()
        self.statusTerminal = statusTerminal()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.velocity_control = VelocityControl(node)
        self.gui_status_sub = node.create_subscription(String, 'gui_status', self.string_callback, 5)
        # self.auto_abort_pub = rospy.Publisher('/auto_abort_check', Bool, queue_size=5)
        # self.next_state_pub = rospy.Publisher('/next_state', Bool, queue_size=5)
        self.science_serial_controller = node.create_publisher( String, '/science_serial_control',5)
        self.science_led_uv_publisher = self.node.create_publisher(Bool, '/science_led_uv', 10)
        self.science_led_blue_publisher = self.node.create_publisher(Bool, '/science_led_blue', 10)
        self.science_position_servo_publisher = self.node.create_publisher(Int32, '/science_position_servo', 10)
        self.science_serial_data = node.create_subscription( String, '/science_serial_data', self.get_probe_data_callback, 10)
        self.optical_data_active = False
        self.optical_data_x = []
        self.optical_data_y = []
        self.optical_sample_index = 0
        self.optical_data_subscriber = None

        # Adding stuff for chem+temp tab
        # TODO: confirm topic name with science team
        self.chem_servo_pub = self.node.create_publisher(String, '/chem_servo_control', 10)
        # TODO: confirm topic name with science team  
        self.chem_image_sub = self.node.create_subscription
        (
            CompressedImage,
            '/chem_image',               # TODO: confirm topic name
            self.chem_image_callback,
            10
        )
        # TODO: confirm topic names with science team
        self.site1_temp_sub = self.node.create_subscription(
            String,
            '/science_temp_site1',      # TODO: confirm topic name
            self.site1_temp_callback,
            10
        )
        self.site1_hum_sub = self.node.create_subscription(
            String,
            '/science_hum_site1',       # TODO: confirm topic name
            self.site1_hum_callback,
            10
        )
        self.site2_temp_sub = self.node.create_subscription(
            String,
            '/science_temp_site2',      # TODO: confirm topic name
            self.site2_temp_callback,
            10
        )

        self.site2_hum_sub = self.node.create_subscription(
            String,
            '/science_hum_site2',       # TODO: confirm topic name
            self.site2_hum_callback,
            10
        )
        # These variables will probably track whether sites are stopped
        self.site1_stopped = False
        self.site2_stopped = False
        self.chem_image_active = False

        # Store data for plotting
        self.site1_temp_data = []
        self.site1_temp_time = []
        self.site1_hum_data = []
        self.site1_hum_time = []
        self.site2_temp_data = []
        self.site2_temp_time = []
        self.site2_hum_data = []
        self.site2_hum_time = []

        # Time counters for each graph
        self.site1_temp_counter = 0
        self.site1_hum_counter = 0
        self.site2_temp_counter = 0
        self.site2_hum_counter = 0

        #----------------------------- End of Chem Tab changes

        self.reached_state = None
        self.ph1_plot_data = []
        self.ph2_plot_data = []
        self.hum_plot_data = []
        self.temp_plot_data = []
        self.pmt_plot_data = []
        self.ph1_time, self.ph2_time, self.hum_time, self.temp_time, self.pmt_time = (1, 1, 1, 1, 1)
        # self.led_uv = 0
        # self.science_led_uv_publisher.publish(self.led_uv)
        # self.led_blue = 0
        # self.science_led_blue_publisher.publish(self.led_blue)


        # Add save data variables for each measurement type
        self.ph1_save_data = []  # For storing finalized pH1 data
        self.ph2_save_data = []  # For storing finalized pH2 data
        self.hum_save_data = []  # For storing finalized humidity data
        self.temp_save_data = []  # For storing finalized temperature data
        self.pmt_save_data = []  # For storing finalized PMT data

        self.ms_bridge = CvBridge()
        self.ms_wavelength_sequence = [None, 440, 500, 530, 570, 610, 670, 740, 780, 840, 900, 950, 1000]
        self.ms_collect_queue = []
        self.ms_expected_images = 0
        self.ms_collection_mode = None
        self.ms_session_dir = None
        self.ms_metadata_path = None
        self.ms_entries = []
        self.ms_selected_pixel = None
        self.ms_last_latitude = None
        self.ms_last_longitude = None
        self.ms_last_heading = None

        self.ms_data_sites_root = Path(__file__).parent.resolve() / "data sites"
        self.ms_data_site_dirs = []

        self.ms_gps_sub = node.create_subscription(NavSatFix, '/calian_gnss/gps', self.multispectral_gps_callback, 10)
        self.ms_heading_sub = node.create_subscription(GnssSignalStatus, '/calian_gnss/gps_extended', self.multispectral_heading_callback, 10)
        self.ms_image_sub = node.create_subscription(Image, '/geniecam', self.multispectral_image_callback, 10)

        # Create tabs
        self.scienceTab = QWidget()  # Create science tab first
        self.longlat_tab = QWidget()
        self.controlTab = QWidget()
        self.camsTab = QWidget()
        self.camerasLauncherTab = QWidget()
        # New Tabs (2026 Revision)
        self.chemTempTab = QWidget()
        self.multispectralTab = QWidget()
        self.opticalTab = QWidget()

        self.cameras_window = CamerasWindow(self.node, self)

        # Setup tabs before adding them
        self.setup_science_tab()  # Setup science tab first
        self.setup_lngLat_tab()
        self.setup_control_tab()
        self.setup_cams_tab()
        self.setup_cameras_launcher_tab()
        self.setup_chem_temp_tab()
        self.setup_multispectral_tab()
        self.setup_optical_tab()
        
        # Add tabs to QTabWidget - with science tab first
        self.tabs.addTab(self.scienceTab, "Science")  # Science tab is now first/default
        self.tabs.addTab(self.longlat_tab, "State Machine")
        self.tabs.addTab(self.controlTab, "Controls")
        self.tabs.addTab(self.camsTab, "Cameras")
        self.tabs.addTab(self.camerasLauncherTab, "cameras")
        self.tabs.addTab(self.chemTempTab, "Chem +Temp/Humidity")
        self.tabs.addTab(self.multispectralTab, "Multispectral")
        self.tabs.addTab(self.opticalTab, "Optical")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)
        
        self.statusSignal.connect(self.string_signal_receive)
        self.probeUpdateSignal.connect(self.update_science_plot)
        self.multispectralImageSavedSignal.connect(self.multispectral_on_image_saved)
        self.multispectralStatusSignal.connect(self.multispectral_set_status)

    def setup_cameras_launcher_tab(self):
        layout = QVBoxLayout()

        info_label = QLabel("Open the second cameras window")
        info_label.setAlignment(Qt.AlignCenter)

        self.open_cameras_window_button = QPushButton("Open Cameras Window")
        self.open_cameras_window_button.clicked.connect(self.open_cameras_window)

        layout.addWidget(info_label)
        layout.addWidget(self.open_cameras_window_button)
        layout.addStretch()

        self.camerasLauncherTab.setLayout(layout)

    def open_cameras_window(self):
        self.cameras_window.show()
        self.cameras_window.raise_()
        self.cameras_window.activateWindow()

    def string_callback(self, msg):
        self.statusTerminal.string_callback(msg)
        self.statusSignal.emit(msg.data)

    def string_signal_receive(self, msg):
        msg_list = msg.split(" ")
        goal_reached_msg = ["Goal", "Point", "Reached:"]
        reached = True
        for i in range(len(goal_reached_msg)):
            try:
                if msg_list[i] != goal_reached_msg[i]:
                    reached = False
                    break
            except IndexError:
                reached = False
                break
        if reached:
            self.reached_state = msg_list[3]
        else:
            self.reached_state = None
        if self.reached_state is not None:
            self.setStyleSheet("background-color: #adebb2")
            self.status_label.setText(f"Goal Reached: {self.reached_state}")
        


        
    def on_tab_changed(self, index):
        tab_name = self.tabs.tabText(index)

        if tab_name == "cameras":
            self.open_cameras_window()
        elif index == 1:
            print("map tab")
        elif index == 2:
            print("split tab")

    # Chem + Temp tab function
    def setup_chem_temp_tab(self):
        main_layout = QVBoxLayout() # I am just setting an outer container to stacks the chem + temp portion vertically
        temp_hum_group = QGroupBox("Temp/Humidity")   # top section
        chem_group = QGroupBox("Chem")                # bottom section
        temp_hum_layout = QVBoxLayout()
        chem_layout = QVBoxLayout()
        

    # Temp Portion Layout
        stop_buttons_layout = QHBoxLayout()

        self.stop_site1_button = QPushButton("Stop Site 1")  
        self.stop_site2_button = QPushButton("Stop Site 2")  
    
        # Adding buttons to the horizontal layout
        stop_buttons_layout.addWidget(self.stop_site1_button)
        stop_buttons_layout.addWidget(self.stop_site2_button)
    
        # Adding the buttons row to the temp/humidity layout
        temp_hum_layout.addLayout(stop_buttons_layout)

        # Now placing all the graphs
        graphs_layout = QHBoxLayout()
        # Site 1 column
        site1_layout = QVBoxLayout()
        site1_label = QLabel("Site 1")
        self.site1_temp_plot = pg.PlotWidget() # Previously pyqtgraph has been already imported as pg
        self.site1_hum_plot = pg.PlotWidget()

        # Site 2 column - temp on top, humidity below
        site2_layout = QVBoxLayout()
        site2_label = QLabel("Site 2")
        self.site2_temp_plot = pg.PlotWidget()
        self.site2_hum_plot = pg.PlotWidget()

        # Now styling all the graphs (just setting the plot background to white)
        for plot in [self.site1_temp_plot, self.site1_hum_plot, self.site2_temp_plot, self.site2_hum_plot]:
            plot.setBackground("w")
            plot.showGrid(x=True, y=True)
        # Labelling each axis
        # Label each graph
        self.site1_temp_plot.setLabel("left", "Temperature (°C)")
        self.site1_temp_plot.setLabel("bottom", "Time")
        self.site1_hum_plot.setLabel("left", "Humidity (%)")
        self.site1_hum_plot.setLabel("bottom", "Time")
        self.site2_temp_plot.setLabel("left", "Temperature (°C)")
        self.site2_temp_plot.setLabel("bottom", "Time")
        self.site2_hum_plot.setLabel("left", "Humidity (%)")
        self.site2_hum_plot.setLabel("bottom", "Time")

        # Now building all the layout
        site1_layout.addWidget(site1_label)
        site1_layout.addWidget(self.site1_temp_plot)
        site1_layout.addWidget(self.site1_hum_plot)
        site2_layout.addWidget(site2_label)
        site2_layout.addWidget(self.site2_temp_plot)
        site2_layout.addWidget(self.site2_hum_plot)
        # Putting the columns side by side
        graphs_layout.addLayout(site1_layout)
        graphs_layout.addLayout(site2_layout)
        # Adding everything back to temp_hum section
        temp_hum_layout.addLayout(graphs_layout)
    # Chem Section
        chem_layout = QVBoxLayout()
    
        # Row 1 - Servo buttons side by side
        servo_buttons_layout = QHBoxLayout()
        
        self.servo_back_button = QPushButton("Servo Backward")
        self.servo_forward_button = QPushButton("Servo Forward")
        
        servo_buttons_layout.addWidget(self.servo_back_button)
        servo_buttons_layout.addWidget(self.servo_forward_button)
        
        # Row 2 It will show display button and image side by side
        display_layout = QHBoxLayout()
        
        self.chem_display_button = QPushButton("Display")
        
        # This label will show the image when received
        self.chem_image_label = QLabel("No Image Yet")
        self.chem_image_label.setAlignment(Qt.AlignCenter)
        self.chem_image_label.setMinimumSize(320, 240)   # min size for image
        self.chem_image_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)  # border
        self.chem_image_label.setStyleSheet("background-color: #808080;")  # grey background
        
        display_layout.addWidget(self.chem_display_button)
        display_layout.addWidget(self.chem_image_label)
        
        # Add both rows to chem layout
        chem_layout.addLayout(servo_buttons_layout)
        chem_layout.addLayout(display_layout)

        # Assigning layouts to group boxes
        temp_hum_group.setLayout(temp_hum_layout)
        chem_group.setLayout(chem_layout)
        
        # Add both group boxes to main layout (stacked vertically)
        main_layout.addWidget(temp_hum_group)
        main_layout.addWidget(chem_group)
        
        # Set the main layout on the actual tab
        self.chemTempTab.setLayout(main_layout)


        # I am gonna connect buttons to functions
        # Temp Section
        self.stop_site1_button.clicked.connect(self.stop_site1_graphs)
        self.stop_site2_button.clicked.connect(self.stop_site2_graphs)
        # Chem Section
        self.servo_back_button.clicked.connect(self.chem_servo_backward)
        self.servo_forward_button.clicked.connect(self.chem_servo_forward)
        self.chem_display_button.clicked.connect(self.toggle_chem_image)

    def setup_multispectral_tab(self):
        root_layout = QVBoxLayout()

        top_layout = QHBoxLayout()

        data_group = QGroupBox("Data Sites")
        data_layout = QVBoxLayout()

        self.ms_image_list = QListWidget()
        self.ms_image_list.setMinimumWidth(260)

        self.multispectral_create_data_site_folders()
        self.multispectral_populate_data_sites_list()

        data_layout.addWidget(self.ms_image_list)
        data_group.setLayout(data_layout)

        image_group = QGroupBox("Image Viewer")
        image_layout = QVBoxLayout()

        image_header_layout = QHBoxLayout()
        image_header_layout.addStretch(1)
        self.ms_current_wavelength_label = QLabel("λ = ?")
        image_header_layout.addWidget(self.ms_current_wavelength_label)

        self.ms_image_label = PixelSelectableLabel()
        self.ms_image_label.setMinimumSize(650, 420)
        self.ms_image_label.setStyleSheet("background-color: #202020; border: 1px solid #808080;")

        self.ms_image_scroll = QScrollArea()
        self.ms_image_scroll.setWidget(self.ms_image_label)
        self.ms_image_scroll.setWidgetResizable(False)
        self.ms_image_scroll.setAlignment(Qt.AlignCenter)

        nav_layout = QHBoxLayout()
        self.ms_prev_button = QPushButton("<")
        self.ms_next_button = QPushButton(">")
        nav_layout.addStretch(1)
        nav_layout.addWidget(self.ms_prev_button)
        nav_layout.addWidget(self.ms_next_button)
        nav_layout.addStretch(1)

        zoom_layout = QHBoxLayout()
        self.ms_zoom_slider = QSlider(Qt.Horizontal)
        self.ms_zoom_slider.setMinimum(25)
        self.ms_zoom_slider.setMaximum(400)
        self.ms_zoom_slider.setValue(100)
        self.ms_zoom_value_label = QLabel("100%")
        zoom_layout.addWidget(QLabel("Zoom"))
        zoom_layout.addWidget(self.ms_zoom_slider)
        zoom_layout.addWidget(self.ms_zoom_value_label)

        self.ms_pixel_label = QLabel("Pixel: (not selected)")

        image_layout.addLayout(image_header_layout)
        image_layout.addWidget(self.ms_image_scroll)
        image_layout.addLayout(nav_layout)
        image_layout.addLayout(zoom_layout)
        image_layout.addWidget(self.ms_pixel_label)
        image_group.setLayout(image_layout)

        top_layout.addWidget(data_group, 1)
        top_layout.addWidget(image_group, 3)

        bottom_layout = QHBoxLayout()

        control_group = QGroupBox("Controls")
        control_layout = QGridLayout()
        self.ms_collect_13_button = QPushButton("Collect 13")
        self.ms_collect_single_button = QPushButton("Collect Single")
        self.ms_generate_graph_button = QPushButton("Generate Graph")
        self.ms_servo_backward_button = QPushButton("Servo Backward")
        self.ms_servo_forward_button = QPushButton("Servo Forward")

        self.ms_single_wavelength_combo = QComboBox()
        for wavelength in self.ms_wavelength_sequence:
            if wavelength is None:
                self.ms_single_wavelength_combo.addItem("None")
            else:
                self.ms_single_wavelength_combo.addItem(f"{wavelength} nm")

        control_layout.addWidget(self.ms_collect_13_button, 0, 0)
        control_layout.addWidget(self.ms_collect_single_button, 0, 1)
        control_layout.addWidget(QLabel("Single wavelength:"), 1, 0)
        control_layout.addWidget(self.ms_single_wavelength_combo, 1, 1)
        control_layout.addWidget(self.ms_generate_graph_button, 2, 0, 1, 2)
        control_layout.addWidget(self.ms_servo_backward_button, 3, 0)
        control_layout.addWidget(self.ms_servo_forward_button, 3, 1)
        control_group.setLayout(control_layout)

        graph_group = QGroupBox("Reflectance Graph")
        graph_layout = QVBoxLayout()
        self.ms_graph = pg.PlotWidget()
        self.ms_graph.setBackground("w")
        self.ms_graph.showGrid(x=True, y=True, alpha=0.3)
        self.ms_graph.setLabel("left", "Greyscale Value")
        self.ms_graph.setLabel("bottom", "Filter Index")
        self.ms_graph.setYRange(0, 255)
        graph_layout.addWidget(self.ms_graph)
        graph_group.setLayout(graph_layout)

        bottom_layout.addWidget(control_group, 1)
        bottom_layout.addWidget(graph_group, 2)

        self.ms_gps_label = QLabel("GPS: lat ?, lon ?, heading ?")
        self.ms_status_label = QLabel("Status: Ready")

        root_layout.addLayout(top_layout)
        root_layout.addLayout(bottom_layout)
        root_layout.addWidget(self.ms_gps_label)
        root_layout.addWidget(self.ms_status_label)
        self.multispectralTab.setLayout(root_layout)

        self.ms_servo_forward_button.clicked.connect(self.multispectral_move_servo_forward)
        self.ms_servo_backward_button.clicked.connect(self.multispectral_move_servo_backward)
        self.ms_collect_13_button.clicked.connect(self.multispectral_collect_13)
        self.ms_collect_single_button.clicked.connect(self.multispectral_collect_single)
        self.ms_generate_graph_button.clicked.connect(self.multispectral_generate_graph)
        self.ms_image_list.currentRowChanged.connect(self.multispectral_display_selected_image)
        self.ms_zoom_slider.valueChanged.connect(self.multispectral_zoom_changed)
        self.ms_image_label.pixelSelected.connect(self.multispectral_pixel_selected)
        self.ms_prev_button.clicked.connect(self.multispectral_prev_image)
        self.ms_next_button.clicked.connect(self.multispectral_next_image)

    def multispectral_create_data_site_folders(self):
        """
        Create the main 'data sites' folder and the 10 data site folders.
        If they already exist, nothing bad happens.
        """
        self.ms_data_sites_root.mkdir(parents=True, exist_ok=True)

        self.ms_data_site_dirs = []

        for i in range(1, 11):
            site_dir = self.ms_data_sites_root / f"data site {i}"
            site_dir.mkdir(parents=True, exist_ok=True)
            self.ms_data_site_dirs.append(site_dir)

    def multispectral_populate_data_sites_list(self):
        """
        Show data site folders in the Data Sites list.
        """
        self.ms_image_list.clear()

        for site_dir in self.ms_data_site_dirs:
            item = QListWidgetItem(site_dir.name)
            item.setData(Qt.UserRole, str(site_dir))
            self.ms_image_list.addItem(item)

    def multispectral_set_status(self, text: str):
        if hasattr(self, 'ms_status_label'):
            self.ms_status_label.setText(f"Status: {text}")

    def multispectral_gps_callback(self, msg):
        self.ms_last_latitude = msg.latitude
        self.ms_last_longitude = msg.longitude
        if hasattr(self, 'ms_gps_label'):
            heading_text = "?" if self.ms_last_heading is None else f"{self.ms_last_heading:.2f}"
            self.ms_gps_label.setText(
                f"GPS: lat {self.ms_last_latitude:.6f}, lon {self.ms_last_longitude:.6f}, heading {heading_text}"
            )

    def multispectral_heading_callback(self, msg):
        self.ms_last_heading = msg.heading
        if hasattr(self, 'ms_gps_label'):
            lat_text = "?" if self.ms_last_latitude is None else f"{self.ms_last_latitude:.6f}"
            lon_text = "?" if self.ms_last_longitude is None else f"{self.ms_last_longitude:.6f}"
            self.ms_gps_label.setText(
                f"GPS: lat {lat_text}, lon {lon_text}, heading {self.ms_last_heading:.2f}"
            )

    def multispectral_send_science_command(self, command: str):
        msg = String()
        msg.data = command
        self.science_serial_controller.publish(msg)

    def multispectral_new_session(self, session_prefix: str):
        base_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data", "multispectral")
        os.makedirs(base_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.ms_session_dir = os.path.join(base_dir, f"{session_prefix}_{timestamp}")
        os.makedirs(self.ms_session_dir, exist_ok=True)
        self.ms_metadata_path = os.path.join(self.ms_session_dir, "metadata.csv")
        with open(self.ms_metadata_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["index", "filename", "wavelength", "latitude", "longitude", "heading", "timestamp"])

    def multispectral_reset_session(self):
        self.ms_entries = []
        self.ms_selected_pixel = None
        self.ms_graph.clear()
        self.ms_pixel_label.setText("Pixel: (not selected)")

        # Keep the data site folders visible
        self.multispectral_populate_data_sites_list()

    def multispectral_wavelength_text(self, wavelength):
        return "None" if wavelength is None else f"{wavelength}nm"

    def multispectral_format_float(self, value, precision=6):
        if value is None:
            return "nan"
        return f"{value:.{precision}f}"

    def multispectral_move_servo_forward(self):
        self.multispectral_send_science_command("<MS_SERVO_FORWARD_STEP>")
        self.multispectralStatusSignal.emit("Servo moved forward by 1 step")

    def multispectral_move_servo_backward(self):
        self.multispectral_send_science_command("<MS_SERVO_BACKWARD_STEP>")
        self.multispectralStatusSignal.emit("Servo moved backward by 1 step")

    def multispectral_collect_13(self):
        if self.ms_expected_images > 0:
            self.multispectralStatusSignal.emit("Collection already in progress")
            return
        self.multispectral_new_session("collect13")
        self.multispectral_reset_session()
        self.ms_collect_queue = self.ms_wavelength_sequence.copy()
        self.ms_expected_images = len(self.ms_collect_queue)
        self.ms_collection_mode = "collect13"
        self.multispectral_send_science_command("<MS_COLLECT_13>")
        self.multispectralStatusSignal.emit(f"Collect 13 started. Saving to {self.ms_session_dir}")

    def multispectral_collect_single(self):
        if self.ms_expected_images > 0:
            self.multispectralStatusSignal.emit("Collection already in progress")
            return
        selected_text = self.ms_single_wavelength_combo.currentText()
        wavelength = None if selected_text == "None" else int(selected_text.replace(" nm", ""))
        self.multispectral_new_session("single")
        self.multispectral_reset_session()
        self.ms_collect_queue = [wavelength]
        self.ms_expected_images = 1
        self.ms_collection_mode = "single"
        self.multispectral_send_science_command("<MS_COLLECT_SINGLE>")
        self.multispectralStatusSignal.emit(
            f"Collect single started ({self.multispectral_wavelength_text(wavelength)}). Saving to {self.ms_session_dir}"
        )

    def multispectral_image_callback(self, msg):
        if self.ms_expected_images <= 0 or not self.ms_collect_queue or self.ms_session_dir is None:
            return
        try:
            cv_image = self.ms_bridge.imgmsg_to_cv2(msg)
            if len(cv_image.shape) == 2:
                gray = cv_image
                bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                rgb = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            else:
                if cv_image.shape[2] == 4:
                    bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
                else:
                    bgr = cv_image
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            wavelength = self.ms_collect_queue.pop(0)
            image_index = len(self.ms_entries) + 1
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            latitude = self.ms_last_latitude
            longitude = self.ms_last_longitude
            heading = self.ms_last_heading

            filename = (
                f"{image_index:02d}_{self.multispectral_wavelength_text(wavelength)}"
                f"_lat_{self.multispectral_format_float(latitude, 6)}"
                f"_lon_{self.multispectral_format_float(longitude, 6)}"
                f"_head_{self.multispectral_format_float(heading, 2)}.png"
            )
            filepath = os.path.join(self.ms_session_dir, filename)
            cv2.imwrite(filepath, bgr)

            with open(self.ms_metadata_path, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    image_index,
                    filename,
                    self.multispectral_wavelength_text(wavelength),
                    self.multispectral_format_float(latitude, 6),
                    self.multispectral_format_float(longitude, 6),
                    self.multispectral_format_float(heading, 2),
                    timestamp,
                ])

            entry = {
                "index": image_index,
                "filepath": filepath,
                "wavelength": wavelength,
                "wavelength_label": self.multispectral_wavelength_text(wavelength),
                "latitude": latitude,
                "longitude": longitude,
                "heading": heading,
                "gray": gray,
                "rgb": rgb,
            }
            self.ms_entries.append(entry)
            self.ms_expected_images -= 1
            self.multispectralImageSavedSignal.emit(entry)

            if self.ms_expected_images == 0:
                self.ms_collection_mode = None
                self.multispectralStatusSignal.emit(
                    f"Collection complete. Saved {len(self.ms_entries)} image(s) to {self.ms_session_dir}"
                )
            else:
                self.multispectralStatusSignal.emit(
                    f"Captured {len(self.ms_entries)} image(s). Waiting for {self.ms_expected_images} more"
                )
        except Exception as error:
            self.ms_expected_images = 0
            self.ms_collect_queue = []
            self.ms_collection_mode = None
            self.multispectralStatusSignal.emit(f"Capture failed: {error}")

    def multispectral_on_image_saved(self, entry: dict):
        latitude = self.multispectral_format_float(entry["latitude"], 6)
        longitude = self.multispectral_format_float(entry["longitude"], 6)
        heading = self.multispectral_format_float(entry["heading"], 2)
        item_text = (
            f"{entry['index']:02d} | {entry['wavelength_label']} | "
            f"lat {latitude}, lon {longitude}, head {heading}"
        )
        self.ms_image_list.addItem(item_text)
        if self.ms_image_list.count() == 1:
            self.ms_image_list.setCurrentRow(0)

    def multispectral_display_selected_image(self, row: int):
        if row < 0 or row >= len(self.ms_entries):
            return
        entry = self.ms_entries[row]
        self.ms_image_label.set_image_array(entry["rgb"])
        self.ms_current_wavelength_label.setText(f"λ = {entry['wavelength_label']}")
        self.multispectral_zoom_changed(self.ms_zoom_slider.value())

    def multispectral_prev_image(self):
        if self.ms_image_list.count() == 0:
            return
        row = self.ms_image_list.currentRow()
        self.ms_image_list.setCurrentRow(max(0, row - 1))

    def multispectral_next_image(self):
        count = self.ms_image_list.count()
        if count == 0:
            return
        row = self.ms_image_list.currentRow()
        self.ms_image_list.setCurrentRow(min(count - 1, row + 1))

    def multispectral_zoom_changed(self, value: int):
        self.ms_zoom_value_label.setText(f"{value}%")
        self.ms_image_label.set_zoom(value / 100.0)

    def multispectral_pixel_selected(self, x: int, y: int):
        self.ms_selected_pixel = (x, y)
        self.ms_pixel_label.setText(f"Pixel: ({x}, {y})")
        self.multispectral_plot_pixel(x, y)

    def multispectral_generate_graph(self):
        if self.ms_selected_pixel is None:
            self.multispectralStatusSignal.emit("Select a pixel first by clicking on the image")
            return
        x, y = self.ms_selected_pixel
        self.multispectral_plot_pixel(x, y)

    def multispectral_plot_pixel(self, x: int, y: int):
        if not self.ms_entries:
            self.multispectralStatusSignal.emit("No images collected yet")
            return

        x_values = []
        y_values = []

        for index, entry in enumerate(self.ms_entries, start=1):
            if index > 13:
                break
            gray = entry["gray"]
            height, width = gray.shape[:2]
            if 0 <= x < width and 0 <= y < height:
                greyscale_value = int(gray[y, x])
            else:
                greyscale_value = 0
            x_values.append(index)
            y_values.append(greyscale_value)

        self.ms_graph.clear()
        pen = pg.mkPen(color=(0, 0, 255), width=2)
        self.ms_graph.plot(x_values, y_values, pen=pen, symbol='o', symbolSize=8, symbolBrush='b')
        self.ms_graph.getAxis('bottom').setTicks([[(i, str(i)) for i in range(1, 14)]])
        self.ms_graph.setXRange(1, 13, padding=0)
        self.ms_graph.setYRange(0, 255)
        self.multispectralStatusSignal.emit(f"Graph updated for pixel ({x}, {y})")

    # defining some functions for the chem+temp tab
    def stop_site1_graphs(self):
        self.site1_stopped = True
        self.stop_site1_button.setEnabled(False)
        self.stop_site1_button.setText("Site 1 Stopped")
        print("Site 1 graphs stopped")
    def stop_site2_graphs(self):
        self.site2_stopped = True
        self.stop_site2_button.setEnabled(False)
        self.stop_site2_button.setText("Site 2 Stopped")
        print("Site 2 graphs stopped")
    def chem_servo_forward(self):
        msg = String()
        msg.data = "<CHEM_SERVO_FORWARD>"  # TODO: confirm this command with science team
        self.chem_servo_pub.publish(msg)
        print("Chem servo forward")

    def chem_servo_backward(self):
        msg = String()
        msg.data = "<CHEM_SERVO_BACKWARD>"  # TODO: confirm this command with science team
        self.chem_servo_pub.publish(msg)
        print("Chem servo backward")

    def toggle_chem_image(self):
        # Here I will Subscribe to chem image topic when Display is clicked
        self.chem_image_active = not self.chem_image_active
        if self.chem_image_active:
            self.chem_display_button.setStyleSheet("background-color: #FF0000;")
            print("Chem image display ON")
        else:
            self.chem_display_button.setStyleSheet("")
            print("Chem image display OFF")
    
    #Adding Optical Module tab
    def setup_optical_tab(self):
        optical_main_layout = QVBoxLayout()

        # Top control row
        top_controls_group = QGroupBox("Optical Controls")
        top_controls_layout = QHBoxLayout()

        self.optical_servo_backward_button = QPushButton("Servo Backward")
        self.optical_servo_forward_button = QPushButton("Servo Forward")
        self.led_uv_button = QPushButton("UV LED Off")
        self.led_blue_button = QPushButton("Blue LED Off")

        self.led_uv_button.setCheckable(True)
        self.led_blue_button.setCheckable(True)

        self.led_uv_button.setStyleSheet("""
            QPushButton:checked {
                background-color: purple;
                color: white;
            }
        """)

        self.led_blue_button.setStyleSheet("""
            QPushButton:checked {
                background-color: blue;
                color: white;
            }
        """)

        top_controls_layout.addWidget(self.optical_servo_backward_button)
        top_controls_layout.addWidget(self.optical_servo_forward_button)
        top_controls_layout.addSpacing(20)
        top_controls_layout.addWidget(self.led_uv_button)
        top_controls_layout.addWidget(self.led_blue_button)
        top_controls_layout.addStretch()

        top_controls_group.setLayout(top_controls_layout)

        # Data controls
        data_group = QGroupBox("Data")
        data_layout = QHBoxLayout()

        self.optical_data_button = QPushButton("Start Data")
        self.optical_data_button.setCheckable(True)
        self.optical_data_button.setStyleSheet("""
            QPushButton {
                padding: 6px 12px;
            }
            QPushButton:checked {
                background-color: orange;
                color: black;
                font-weight: bold;
            }
        """)

        data_layout.addWidget(self.optical_data_button)
        data_layout.addStretch()
        data_group.setLayout(data_layout)

        # Graph
        graph_group = QGroupBox("Graph")
        graph_layout = QHBoxLayout()

        self.optical_graph = pg.PlotWidget()
        self.optical_graph.setBackground("w")
        self.optical_graph.showGrid(x=True, y=True, alpha=0.3)
        self.optical_graph.setLabel("left", "Y Value")
        self.optical_graph.setLabel("bottom", "Sample Index")

        graph_layout.addWidget(self.optical_graph)
        graph_group.setLayout(graph_layout)

        optical_main_layout.addWidget(top_controls_group)
        optical_main_layout.addWidget(data_group)
        optical_main_layout.addWidget(graph_group)

        self.opticalTab.setLayout(optical_main_layout)

        self.optical_servo_backward_button.clicked.connect(self.optical_servo_backward)
        self.optical_servo_forward_button.clicked.connect(self.optical_servo_forward)
        self.led_uv_button.clicked.connect(self.led_uv_btnstate)
        self.led_blue_button.clicked.connect(self.led_blue_btnstate)
        self.optical_data_button.clicked.connect(self.start_display_graph)

    def led_uv_btnstate(self):
        if self.led_uv_button.isChecked():
            self.led_uv = 1
            self.led_uv_button.setText("UV LED On")
        else:
            self.led_uv = 0
            self.led_uv_button.setText("UV LED Off")
        msg = Bool()
        msg.data = self.led_uv  # TODO: confirm this command with science team
        self.science_led_uv_publisher.publish(msg)

    def led_uv_btnstate(self):
        if self.led_uv_button.isChecked():
            self.led_uv = 1
            self.led_uv_button.setText("UV LED On")
        else:
            self.led_uv = 0
            self.led_uv_button.setText("UV LED Off")
        msg = Bool()
        msg.data = self.led_uv
        self.science_led_uv_publisher.publish(msg)

    def led_blue_btnstate(self):
        if self.led_blue_button.isChecked():
            self.led_blue = 1
            self.led_blue_button.setText("Blue LED On")
        else:
            self.led_blue = 0
            self.led_blue_button.setText("Blue LED Off")
        msg = Bool()
        msg.data = self.led_blue
        self.science_led_blue_publisher.publish(msg)

    def optical_servo_forward(self):
        msg = Int32()
        msg.data = 1   # TODO: change if your servo command value is different
        self.science_position_servo_publisher.publish(msg)
        print("Optical servo forward")

    def optical_servo_backward(self):
        msg = Int32()
        msg.data = -1  # TODO: change if your servo command value is different
        self.science_position_servo_publisher.publish(msg)
        print("Optical servo backward")

    def stop_site1_graphs(self):
        msg = Int32()
        msg.data = 1 #TODO: confirm this command with science team
        self.science_position_servo_publisher.publish(msg)

    def stop_site2_graphs(self):
        msg = Int32()
        msg.data = -1 #TODO: confirm this command with science team
        self.science_position_servo_publisher.publish(msg)

    #MODIFY
    def start_display_graph(self):
        if self.optical_data_active:
            self.stop_display_graph()
            return

        # Start a fresh run
        self.optical_graph.clear()
        self.optical_data_x = []
        self.optical_data_y = []
        self.optical_sample_index = 0

        self.optical_data_subscriber = self.node.create_subscription(
            Float32MultiArray,
            '/optical_data',   # TODO: replace if needed
            self.optical_data_callback,
            10
        )

        self.optical_data_active = True
        self.optical_data_button.setChecked(True)
        self.optical_data_button.setText("Stop Data")
    
    def optical_data_callback(self, msg):
        if not self.optical_data_active:
            return

        if msg.data is None or len(msg.data) == 0:
            return

        # x is constant progression/index, y is the changing incoming value
        self.optical_sample_index += 1
        self.optical_data_x.append(self.optical_sample_index)
        self.optical_data_y.append(float(msg.data[0]))

        self.optical_plot_data()

    def optical_plot_data(self):
        if len(self.optical_data_y) == 0:
            print("No data collected yet for optical page")
            return

        self.optical_graph.clear()

        pen = pg.mkPen(color=(0, 0, 255), width=2)
        self.optical_graph.plot(
            self.optical_data_x,
            self.optical_data_y,
            pen=pen,
            symbol='o',
            symbolSize=6,
            symbolBrush='b'
        )

    def stop_display_graph(self):
        self.optical_data_active = False

        # Reset button FIRST so even if saving/unsubscribing has an issue,
        # the UI still goes back to Start Data.
        self.optical_data_button.setChecked(False)
        self.optical_data_button.setText("Start Data")
        self.optical_data_button.update()

        # Destroy ROS2 subscription correctly.
        if self.optical_data_subscriber is not None:
            self.node.destroy_subscription(self.optical_data_subscriber)
            self.optical_data_subscriber = None

        # Save graph image
        try:
            exporter = ImageExporter(self.optical_graph.plotItem)
            exporter.export('optical_data_graph.png')
            print("Optical graph saved to optical_data_graph.png")
        except Exception as e:
            print(f"Could not save optical graph image: {e}")

        # Save data arrays
        try:
            with open('optical_data.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y'])
                for x_val, y_val in zip(self.optical_data_x, self.optical_data_y):
                    writer.writerow([x_val, y_val])

            print("Optical data saved to optical_data.csv")
        except Exception as e:
            print(f"Could not save optical data CSV: {e}")


    # Call Back functions
    def site1_temp_callback(self, msg):  # Called automatically when temperature data arrives for site 1
        if self.site1_stopped:   # if stop button was pressed, ignore new data
            return
        
        value = float(msg.data)              
        self.site1_temp_counter += 1         
        self.site1_temp_data.append(value)   
        self.site1_temp_time.append(self.site1_temp_counter)  
        
        # Update the graph
        self.site1_temp_plot.clear()         
        self.site1_temp_plot.plot(  
            self.site1_temp_time,            # x axis = time
            self.site1_temp_data,            # y axis = temperature
            pen=pg.mkPen(color=(255, 0, 0))  # red line
    )
        
    #Called automatically when humidity data arrives for site 1
    def site1_hum_callback(self, msg):
        if self.site1_stopped:
            return
        
        value = float(msg.data)
        self.site1_hum_counter += 1
        self.site1_hum_data.append(value)
        self.site1_hum_time.append(self.site1_hum_counter)
        
        self.site1_hum_plot.clear()
        self.site1_hum_plot.plot(
            self.site1_hum_time,
            self.site1_hum_data,
            pen=pg.mkPen(color=(0, 0, 255))  # blue line
    )
    def site2_temp_callback(self, msg): # Called automatically when temperature data arrives for site 2
        if self.site2_stopped:
            return
        
        value = float(msg.data)
        self.site2_temp_counter += 1
        self.site2_temp_data.append(value)
        self.site2_temp_time.append(self.site2_temp_counter)
        
        self.site2_temp_plot.clear()
        self.site2_temp_plot.plot(
            self.site2_temp_time,
            self.site2_temp_data,
            pen=pg.mkPen(color=(255, 0, 0))  # red line
    )
    # Called automatically when humidity data arrives for site 2
    def site2_hum_callback(self, msg):
        if self.site2_stopped:
            return
        
        value = float(msg.data)
        self.site2_hum_counter += 1
        self.site2_hum_data.append(value)
        self.site2_hum_time.append(self.site2_hum_counter)
        
        self.site2_hum_plot.clear()
        self.site2_hum_plot.plot(
            self.site2_hum_time,
            self.site2_hum_data,
            pen=pg.mkPen(color=(0, 0, 255))  # blue line
        )
    
    # Called automatically when a new image arrives from science team
    def chem_image_callback(self, msg):
        if not self.chem_image_active:  # only show image if display is ON
            return
        
        # Convert ROS2 compressed image to something Qt can display
        np_arr = np.frombuffer(msg.data, np.uint8)       # convert to numpy array
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # decode image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # fix colors
        
        # Convert to Qt format for display
        height, width, _ = cv_image.shape
        bytes_per_line = 3 * width
        qimg = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        
        # Scale image to fit the label
        scaled_pixmap = pixmap.scaled(
            self.chem_image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        
        # Display the image
        self.chem_image_label.setPixmap(scaled_pixmap)




    def setup_cams_tab(self):
        # Add camera feed to the splitter
        camera_group = QGroupBox("Camera Feed Tabs")
        camera_layout = QVBoxLayout()
        

        self.camera_label1_cams_tab = ResizableLabel()
        self.camera_label1_cams_tab.setMinimumSize(320, 240)
        self.camera_label1_cams_tab.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label1_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        

        self.camera_label2_cams_tab = ResizableLabel()
        self.camera_label2_cams_tab.setMinimumSize(320, 240)
        self.camera_label2_cams_tab.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label2_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label3_cams_tab = ResizableLabel()
        self.camera_label3_cams_tab.setMinimumSize(320, 240)
        self.camera_label3_cams_tab.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label3_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label4_cams_tab = ResizableLabel()
        self.camera_label4_cams_tab.setMinimumSize(320, 240)
        self.camera_label4_cams_tab.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label4_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label5_cams_tab = ResizableLabel()
        self.camera_label5_cams_tab.setMinimumSize(320, 240)
        self.camera_label5_cams_tab.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label5_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ROS functionality
        self.camerasplitter_cams_tab = QSplitter(Qt.Horizontal)
        self.camera_feed_cams_tab = CameraFeed(self.node, self.camera_label1_cams_tab, self.camera_label2_cams_tab, self.camera_label3_cams_tab, self.camera_label4_cams_tab, self.camera_label5_cams_tab, self.camerasplitter_cams_tab)
        self.camerasplitter_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        # Use CameraSelect menu-based selector
        self.camera_selector_cams_tab = CameraSelect()
        self.camera_selector_cams_tab.cameras_changed.connect(self.camera_feed_cams_tab.update_active_cameras)

        camera_layout.addWidget(self.camera_selector_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label1_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label2_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label3_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label4_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label5_cams_tab)
        camera_layout.addWidget(self.camerasplitter_cams_tab)

        # camera_layout.addWidget(self.camera_label_splitter)
        camera_group.setLayout(camera_layout)
        cam_tab_layout = QVBoxLayout()
        # split_screen_layout.addWidget(splitter)
        cam_tab_layout.addWidget(camera_group)
        # split_screen_layout.addWidget(self.statusTermGroupBox) 
        self.camsTab.setLayout(cam_tab_layout)

    def setup_control_tab(self):
        self.controls_group = QGroupBox("Controls")
        controls_layout = QHBoxLayout()

        # Joystick (for Controls tab)
        joystick_group = QGroupBox("Joystick")
        joystick_layout = QVBoxLayout()
        self.joystick_control = Joystick(self.velocity_control)
        self.joystick_control.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        joystick_layout.addWidget(self.joystick_control)
        joystick_group.setLayout(joystick_layout)

        # Gear slider
        gear_group = QGroupBox("Gear Control")
        slider_layout = QVBoxLayout()
        self.gear_slider_control = Slider(Qt.Horizontal)
        self.gear_slider_control.setMinimum(0)
        self.gear_slider_control.setMaximum(100)
        self.gear_slider_control.setValue(0)
        slider_layout.addWidget(self.gear_slider_control)
        gear_group.setLayout(slider_layout)

        #Exposure slider
        exposure_group = QGroupBox("Exposure Control")
        exposure_layout = QVBoxLayout()
        self.exposure_slider_control = Slider(Qt.Horizontal)
        self.exposure_slider_control.setMinimum(0)
        self.exposure_slider_control.setMaximum(100)
        self.exposure_slider_control.setValue(50)
        exposure_layout.addWidget(self.exposure_slider_control)
        exposure_group.setLayout(exposure_layout)

        # Sync joystick movements between tabs
        self.joystick_control.joystickMoved.connect(self.sync_joysticks)

        # Add to layout
        controls_layout.addWidget(joystick_group)
        controls_layout.addWidget(gear_group)
        controls_layout.addWidget(exposure_group)
        self.controls_group.setLayout(controls_layout)

        control_tab_layout = QVBoxLayout()
        control_tab_layout.addWidget(self.controls_group)
        self.controlTab.setLayout(control_tab_layout)

        
        # self.controlTab.setLayout(control_tab_layout)

    def setup_lngLat_tab(self):
        self.lngLatEntry = LngLatEntryBar(self.node, self.map_overlay)
        self.lngLatFile = LngLatEntryFromFile(self.node, self.map_overlay)
        self.lngLatDeliveryFile = LngLatDeliveryEntryFromFile(self.map_overlay)
        self.stateMachineDialog = StateMachineStatus(self.node)
        self.arucoBox = ArucoWidget(self.node)
        

        # Create a group box for the ArucoWidget
        self.arucoGroupBox = QGroupBox("Aruco Tags")
        aruco_layout = QVBoxLayout()
        aruco_layout.addWidget(self.arucoBox)
        self.arucoGroupBox.setLayout(aruco_layout)

        

        # Main layout for the tab
        Lnglat_tab_layout = QVBoxLayout()
        Lnglat_tab_layout.addWidget(self.lngLatEntry)
        Lnglat_tab_layout.addWidget(self.lngLatFile)
        Lnglat_tab_layout.addWidget(self.lngLatDeliveryFile)
        Lnglat_tab_layout.addWidget(self.stateMachineDialog)
        Lnglat_tab_layout.addWidget(self.arucoGroupBox) 
        

        self.longlat_tab.setLayout(Lnglat_tab_layout)
    def sync_joysticks(self, x, y):
        """Sync joystick movement between tabs"""
        self.joystick_splitter.setJoystickPosition(x, y)
        self.joystick_control.setJoystickPosition(x, y)
        # add more joystick instances here to sync

    def setup_science_tab(self):
        self.ph1_start = 0
        self.ph2_start = 0
        self.hum_start = 0
        self.temp_start = 0
        self.pmt_start = 0

        self.pmt_switch = 1

        button_style = "padding: 4px; min-height: 20px;"
        # Create main tab
        self.scienceTab = QWidget()
        
        # Main horizontal splitter dividing the tab into 2 sections
        main_splitter = QSplitter(Qt.Horizontal)
        
        # Left section with vertical splitter
        left_vertical_splitter = QSplitter(Qt.Vertical)
        left_vertical_splitter.setHandleWidth(1)  # Minimize the splitter handle width
        
        # First item in left vertical splitter
        self.left_top_widget = SensorBlock(self)
        self.left_top_widget.setMinimumHeight(40)  # Ensure minimum height
        
        # Second item in left vertical splitter with horizontal splitter
        left_middle_widget = QGroupBox("")
        left_middle_widget.setMaximumHeight(200)  # Set maximum height constraint
        left_middle_splitter = QSplitter(Qt.Horizontal)
        left_middle_splitter.setHandleWidth(1)  # Minimize the splitter handle width
        
        # Add 2 items to the horizontal splitter with reduced height
        self.left_middle_item1 = SampleBlock(True, self)
        self.left_middle_item1.setMaximumHeight(180)  # Limit SampleBlock height
        
        self.left_middle_item2 = SampleBlock(False, self)
        self.left_middle_item2.setMaximumHeight(180)  # Limit SampleBlock height
        
        left_middle_splitter.addWidget(self.left_middle_item1)
        left_middle_splitter.addWidget(self.left_middle_item2)
        
        left_middle_layout = QVBoxLayout()
        left_middle_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        left_middle_layout.setSpacing(0)  # Remove spacing
        left_middle_layout.addWidget(left_middle_splitter)
        left_middle_widget.setLayout(left_middle_layout)

        left_bottom_widget = QGroupBox("")
        left_bottom_layout = QVBoxLayout()
        left_bottom_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        left_bottom_layout.setSpacing(0)  # Remove spacing
        
        # Create a vertical splitter for the plots
        self.science_plots_splitter = QSplitter(Qt.Vertical)
        self.science_plots_splitter.setHandleWidth(1)  # Minimize the splitter handle width
        
        # Create separate plot widgets for each data type
        self.science_temperature_plot = pg.PlotWidget()
        self.science_humidity_plot = pg.PlotWidget()
        self.science_ph1_plot = pg.PlotWidget()
        self.science_ph2_plot = pg.PlotWidget()
        self.science_pmt_plot = pg.PlotWidget()
        
        # Set initial background and styling for all plots
        for plot in [self.science_temperature_plot, self.science_humidity_plot, 
                    self.science_ph1_plot, self.science_ph2_plot, self.science_pmt_plot]:
            plot.setBackground("w")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel("bottom", "Time")
        
        self.science_temperature_plot.setLabel("left", "Temperature (°C)")
        self.science_humidity_plot.setLabel("left", "Humidity (%)")
        self.science_ph1_plot.setLabel("left", "Site 1 pH")
        self.science_ph2_plot.setLabel("left", "Site 2 pH")
        self.science_pmt_plot.setLabel("left", "PMT (V)")

        
        # Add plot widgets to splitter
        self.science_plots_splitter.addWidget(self.science_temperature_plot)
        self.science_plots_splitter.addWidget(self.science_humidity_plot)
        self.science_plots_splitter.addWidget(self.science_ph1_plot)
        self.science_plots_splitter.addWidget(self.science_ph2_plot)
        self.science_plots_splitter.addWidget(self.science_pmt_plot)
        
        # By default, hide all plots initially
        self.science_temperature_plot.hide()
        self.science_humidity_plot.hide()
        self.science_ph1_plot.hide()
        self.science_ph2_plot.hide()
        self.science_pmt_plot.hide()
        
        # Add splitter to layout
        left_bottom_layout.addWidget(self.science_plots_splitter)
        left_bottom_widget.setLayout(left_bottom_layout)
        
        # Remove title margins from all group boxes
        self.left_top_widget.setStyleSheet("QGroupBox { margin-top: 0px; }")
        left_middle_widget.setStyleSheet("QGroupBox { margin-top: 0px; }")
        left_bottom_widget.setStyleSheet("QGroupBox { margin-top: 0px; }")
        self.left_middle_item1.setStyleSheet("QGroupBox { margin-top: 0px; }")
        self.left_middle_item2.setStyleSheet("QGroupBox { margin-top: 0px; }")
        
        # Add widgets to left vertical splitter
        left_vertical_splitter.addWidget(self.left_top_widget)
        left_vertical_splitter.addWidget(left_middle_widget)
        left_vertical_splitter.addWidget(left_bottom_widget)
        
        # Set stretch factors to minimize the middle section height
        left_vertical_splitter.setStretchFactor(0, 1)  # Change from 0 to 1 to give SensorBlock space
        left_vertical_splitter.setStretchFactor(1, 2)  # Middle section - some stretch
        left_vertical_splitter.setStretchFactor(2, 3)  # Bottom section - most space
        
        # Right section with vertical splitter
        right_vertical_splitter = QSplitter(Qt.Vertical)
        
        # Items in right vertical splitter
        right_top_widget = QGroupBox("map")
        right_top_layout = QVBoxLayout()
        
        # Create a separate map overlay for the science tab
        self.map_overlay = mapOverlay(self.node)
        self.map_overlay.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Create science-specific map controls
        self.checkbox_setting = QCheckBox("Recentre map when rover offscreen")
        self.checkbox_setting.setChecked(False)
        self.checkbox_setting.stateChanged.connect(
            lambda state: self.on_checkbox_state_changed(state, self.map_overlay)
        )
        
        self.clear_map_button = QPushButton("Clear Map")
        self.clear_map_button.setStyleSheet(button_style)
        self.clear_map_button.clicked.connect(self.map_overlay.clear_map)

        # Create horizontal layout for checkbox and clear button
        science_checkbox_layout = QHBoxLayout()
        science_checkbox_layout.addWidget(self.checkbox_setting)
        science_checkbox_layout.addStretch(1)
        science_checkbox_layout.addWidget(self.clear_map_button)
        
        # Create a container widget for the checkbox layout
        science_checkbox_container = QWidget()
        science_checkbox_container.setLayout(science_checkbox_layout)
        
        # Add the container and map to the map layout
        right_top_layout.addWidget(science_checkbox_container)
        right_top_layout.addWidget(self.map_overlay)
        right_top_widget.setLayout(right_top_layout)

        # Add camera feed to the splitter
        camera_group = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout()
        

        self.camera_label1 = ResizableLabel()
        self.camera_label1.setMinimumSize(320, 240)
        self.camera_label1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label2 = ResizableLabel()
        self.camera_label2.setMinimumSize(320, 240)
        self.camera_label2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label3 = ResizableLabel()
        self.camera_label3.setMinimumSize(320, 240)
        self.camera_label3.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label4 = ResizableLabel()
        self.camera_label4.setMinimumSize(320, 240)
        self.camera_label4.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label4.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.camera_label5 = ResizableLabel()
        self.camera_label5.setMinimumSize(320, 240)
        self.camera_label5.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.camera_label5.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ROS functionality
        self.camerasplitter = QSplitter(Qt.Horizontal)
        self.camera_feed = CameraFeed(self.node, self.camera_label1, self.camera_label2, self.camera_label3, self.camera_label4, self.camera_label5, self.camerasplitter)
        self.camerasplitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.select_splitter = QSplitter(Qt.Horizontal)
        self.select_splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Change to Minimum
        self.select_splitter.setMaximumHeight(50)  # Set a maximum height

        # Use CameraSelect menu-based selector
        self.camera_selector = CameraSelect()
        self.camera_selector.cameras_changed.connect(self.update_active_cameras_science)
        self.camera_selector.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Ensure this is minimal height

        self.genieControl = GenieControl(self.node, self)
        self.genieControl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Ensure this is minimal height

        self.select_splitter.addWidget(self.camera_selector)
        self.select_splitter.addWidget(self.genieControl)

        camera_layout.addWidget(self.select_splitter)
        self.camerasplitter.addWidget(self.camera_label1)
        self.camerasplitter.addWidget(self.camera_label2)
        self.camerasplitter.addWidget(self.camera_label3)
        self.camerasplitter.addWidget(self.camera_label4)
        self.camerasplitter.addWidget(self.camera_label5)
        camera_layout.addWidget(self.camerasplitter)

        # camera_layout.addWidget(self.camera_label_splitter)
        camera_group.setLayout(camera_layout)
        
        # Add widgets to right vertical splitter
        right_vertical_splitter.addWidget(right_top_widget)
        right_vertical_splitter.addWidget(camera_group)
        
        # Add both vertical splitters to main horizontal splitter
        main_splitter.addWidget(left_vertical_splitter)
        main_splitter.addWidget(right_vertical_splitter)
        
        # Set equal size for main sections
        main_splitter.setSizes([int(self.width()/2), int(self.width()/2)])
        
        # Add main splitter to the tab layout
        science_tab_layout = QVBoxLayout()
        science_tab_layout.addWidget(main_splitter)
        self.scienceTab.setLayout(science_tab_layout)

    def science_callback(self, data):
        pass

    def update_active_cameras_science(self, active_cameras):
        self.camera_feed.update_active_cameras(active_cameras)
        # Update visibility of the genie control based on active cameras
        # if active_cameras["Genie camera"]:
        #     self.genieControl.show_genie_button(True)
        # else:
        #     self.genieControl.show_genie_button(False)
        if active_cameras["Microscope camera"]:
            self.genieControl.show_zoom_controls(True)
        else:
            self.genieControl.show_zoom_controls(False)
        if active_cameras["Zed (front) camera"]:
            self.genieControl.show_pano_button(True)
        else:
            self.genieControl.show_pano_button(False)
        # if active_cameras["Genie camera"]:
        #     self.genieControl.show_save_genie_button(True)
        # else:
        #     self.genieControl.show_save_genie_button(False)

    def on_checkbox_state_changed(self, state,map_overlay):
        if state == Qt.Checked:
            map_overlay.centreOnRover = True
            # Perform actions for the checked state
        else:
            map_overlay.centreOnRover = False
            # Perform actions for the unchecked state

    def change_gear(self, value):
        self.velocity_control.set_gear(value/10+1)
        print(f"Changed to Gear: {value}")
        self.gear_slider_splitter.setValue(value)
    
    def change_exposure(self,value):
        #self.camera_feed_cams_tab.exposure_value = value
        self.camera_feed.exposure_value = value

    # def pub_next_state(self):
    #     self.next_state_pub.publish(True)
    #     self.setStyleSheet("background-color: #FFFFFF")
    #     self.status_label.setText("")

    # def pub_manual_abort(self):
    #     self.manual_abort_pub.publish(True)

    # def pub_auto_abort(self):
    #     self.auto_abort_pub.publish(True)
    #     self.setStyleSheet("background-color: #FFFFFF")
    #     self.status_label.setText("")
    
    def get_probe_data_callback(self, data):
        if "RECEIVED" in data.data or "ERROR" in data.data:
            return
        msgs = data.data.split(";")
        msg = []
        for i in msgs:
            if i == "":
                continue
            msg.append(i.split(":"))

        # PH1 block (no changes needed - this is already correct)
        if self.left_middle_item1.display or self.left_middle_item1.site1_block.reading:
            if self.ph1_time == 1:
                self.ph1_time_buffer = []
                self.ph1_data_buffer = []
            self.ph1_time_buffer.append(self.ph1_time)
            ph1_graph_data = float(msg[0][1])
            self.ph1_data_buffer.append(ph1_graph_data)
            self.ph1_time += 1
            self.ph1_plot_data = [self.ph1_data_buffer, self.ph1_time_buffer]
        if self.left_middle_item1.site1_block.start_read:
            self.ph1_start = len(self.ph1_data_buffer)
            self.left_middle_item1.site1_block.start_read = False
        if not (self.left_middle_item1.site1_block.start_read or self.left_middle_item1.site1_block.reading) and self.left_middle_item1.site1_block.stop_read:
            self.ph1_plot_avg = sum(self.ph1_plot_data[0]) / len(self.ph1_plot_data[0])
            # Store a deep copy of the data for saving
            self.ph1_save_data = [self.ph1_plot_data[0][:], self.ph1_plot_data[1][:]]
            self.ph1_save_data[0].insert(0, self.ph1_plot_avg)
            self.ph1_save_data[1].insert(0, -1)
            # self.ph1_time = 1
            self.left_middle_item1.site1_block.stop_read = False

        # PH2 block - updated to match ph1 pattern
        if self.left_middle_item1.display or self.left_middle_item1.site2_block.reading:
            if self.ph2_time == 1:
                self.ph2_time_buffer = []
                self.ph2_data_buffer = []
            self.ph2_time_buffer.append(self.ph2_time)
            ph2_graph_data = float(msg[1][1])
            self.ph2_data_buffer.append(ph2_graph_data)
            self.ph2_time += 1
            self.ph2_plot_data = [self.ph2_data_buffer, self.ph2_time_buffer]
        if self.left_middle_item1.site2_block.start_read:
            self.ph2_start = len(self.ph2_data_buffer)
            self.left_middle_item1.site2_block.start_read = False
        if not (self.left_middle_item1.site2_block.start_read or self.left_middle_item1.site2_block.reading) and self.left_middle_item1.site2_block.stop_read:
            self.ph2_plot_avg = sum(self.ph2_plot_data[0]) / len(self.ph2_plot_data[0])
            # Store a deep copy of the data for saving
            self.ph2_save_data = [self.ph2_plot_data[0][:], self.ph2_plot_data[1][:]]
            self.ph2_save_data[0].insert(0, self.ph2_plot_avg)
            self.ph2_save_data[1].insert(0, -1)
            # self.ph2_time = 1
            self.left_middle_item1.site2_block.stop_read = False
        
        # Merged HUM & TEMP block
        if self.left_top_widget.display or self.left_top_widget.reading:
            # Initialize buffers if needed for both measurements
            if self.hum_time == 1:
                self.hum_time_buffer = []
                self.hum_data_buffer = []
                self.temp_time_buffer = []
                self.temp_data_buffer = []
            
            # Process humidity data
            self.hum_time_buffer.append(self.hum_time)
            hum_graph_data = float(msg[2][1])
            self.hum_data_buffer.append(hum_graph_data)
            
            # Process temperature data with same time index
            self.temp_time_buffer.append(self.hum_time)  # Use same time counter for both
            temp_graph_data = float(msg[3][1])
            self.temp_data_buffer.append(temp_graph_data)
            
            # Increment shared time counter
            self.hum_time += 1
            self.temp_time = self.hum_time  # Keep temp time in sync
            
            # Create plot data for both
            self.hum_plot_data = [self.hum_data_buffer, self.hum_time_buffer]
            self.temp_plot_data = [self.temp_data_buffer, self.temp_time_buffer]

        # Handle start reading event for both measurements
        if self.left_top_widget.start_read:
            self.hum_start = len(self.hum_data_buffer)
            self.temp_start = len(self.temp_data_buffer)
            self.left_top_widget.start_read = False

        # Handle stop reading event for both measurements
        if not (self.left_top_widget.start_read or self.left_top_widget.reading) and self.left_top_widget.stop_read:
            # Process humidity final data
            if len(self.hum_plot_data[0]) > 0:
                self.hum_plot_avg = sum(self.hum_plot_data[0]) / len(self.hum_plot_data[0])
                # Store a deep copy of the data for saving
                self.hum_save_data = [self.hum_plot_data[0][:], self.hum_plot_data[1][:]]
                self.hum_save_data[0].insert(0, self.hum_plot_avg)
                self.hum_save_data[1].insert(0, -1)
            
            # Process temperature final data
            if len(self.temp_plot_data[0]) > 0:
                self.temp_plot_avg = sum(self.temp_plot_data[0]) / len(self.temp_plot_data[0])
                # Store a deep copy of the data for saving
                self.temp_save_data = [self.temp_plot_data[0][:], self.temp_plot_data[1][:]]
                self.temp_save_data[0].insert(0, self.temp_plot_avg)
                self.temp_save_data[1].insert(0, -1)
            
            # Reset counters and buffers
            # self.hum_time = 1
            # self.temp_time = 1
            self.left_top_widget.stop_read = False

        # PMT block - updated to match ph1 pattern
        if self.left_middle_item2.display or self.left_middle_item2.site1_block.reading:
            if self.pmt_time == 1:
                self.pmt_time_buffer = []
                self.pmt_data_buffer = []
            self.pmt_time_buffer.append(self.pmt_time)
            pmt_graph_data = float(msg[4][1])
            self.pmt_data_buffer.append(pmt_graph_data)
            self.pmt_time += 1
            self.pmt_plot_data = [self.pmt_data_buffer, self.pmt_time_buffer]
        if self.left_middle_item2.site1_block.start_read:
            self.pmt_start = len(self.pmt_data_buffer)
            self.left_middle_item2.site1_block.start_read = False
        if not (self.left_middle_item2.site1_block.start_read or self.left_middle_item2.site1_block.reading) and self.left_middle_item2.site1_block.stop_read:
            self.pmt_plot_avg = sum(self.pmt_plot_data[0]) / len(self.pmt_plot_data[0])
            # Store a deep copy of the data for saving
            self.pmt_save_data = [self.pmt_plot_data[0][:], self.pmt_plot_data[1][:]]
            self.pmt_save_data[0].insert(0, self.pmt_plot_avg)
            self.pmt_save_data[1].insert(0, -1)
            # self.pmt_time = 1
            self.left_middle_item2.site1_block.stop_read = False
        
        self.pmt_switch = int(msg[5][1])
        self.left_middle_item2.pmtWidget.setText(f"PMT: {msg[5][1]}")

        self.probeUpdateSignal.emit(True)

    def plot_science_temperature_data(self, data_buffer, time_buffer):
        pen = pg.mkPen(color=(255, 0, 0))
        self.science_temperature_plot.plot(time_buffer, data_buffer, pen=pen, symbol="+",symbolSize=10, symbolBrush="b")
    
    def plot_science_humidity_data(self, data_buffer, time_buffer):
        pen = pg.mkPen(color=(0, 0, 255))
        self.science_humidity_plot.plot(time_buffer, data_buffer, pen=pen, symbol="+",symbolSize=10, symbolBrush="g")

    def plot_science_ph1_data(self, data_buffer, time_buffer):
        pen = pg.mkPen(color=(255, 0, 0))
        self.science_ph1_plot.plot(time_buffer, data_buffer, pen=pen, symbol="+",symbolSize=10, symbolBrush="b")

    def plot_science_ph2_data(self, data_buffer, time_buffer):
        pen = pg.mkPen(color=(255, 0, 0))
        self.science_ph2_plot.plot(time_buffer, data_buffer, pen=pen, symbol="+",symbolSize=10, symbolBrush="b")

    def plot_science_pmt_data(self, data_buffer, time_buffer):
        pen = pg.mkPen(color=(0, 255, 0))
        self.science_pmt_plot.plot(time_buffer, data_buffer, pen=pen, symbol="+",symbolSize=10, symbolBrush="r")
        
    def update_science_plot(self, update: bool):
        if update:
            # Update pH1 plot
            if self.ph1_plot_data != []:
                self.science_ph1_plot.show()  # Show the plot
                self.science_ph1_plot.clear()
                pen = pg.mkPen(color=(255, 0, 0))
                self.science_ph1_plot.setLabel("left", "pH 1")
                self.science_ph1_plot.setLabel("bottom", "Time")
                self.science_ph1_plot.plot(self.ph1_plot_data[1], self.ph1_plot_data[0], 
                                          pen=pen, symbol="+", symbolSize=10, symbolBrush="b")
            
            # Update pH2 plot
            if self.ph2_plot_data != []:
                self.science_ph2_plot.show()  # Show the plot
                self.science_ph2_plot.clear()
                pen = pg.mkPen(color=(255, 0, 0))
                self.science_ph2_plot.setLabel("left", "pH 2")
                self.science_ph2_plot.setLabel("bottom", "Time")
                self.science_ph2_plot.plot(self.ph2_plot_data[1], self.ph2_plot_data[0], 
                                          pen=pen, symbol="+", symbolSize=10, symbolBrush="b")
            
            # Update humidity plot
            if self.hum_plot_data != []:
                self.science_humidity_plot.show()  # Show the plot
                self.science_humidity_plot.clear()
                pen = pg.mkPen(color=(0, 0, 255))
                self.science_humidity_plot.setLabel("left", "Humidity")
                self.science_humidity_plot.setLabel("bottom", "Time")
                self.science_humidity_plot.plot(self.hum_plot_data[1], self.hum_plot_data[0], 
                                              pen=pen, symbol="+", symbolSize=10, symbolBrush="g")
            
            # Update temperature plot
            if self.temp_plot_data != []:
                self.science_temperature_plot.show()  # Show the plot
                self.science_temperature_plot.clear()
                pen = pg.mkPen(color=(255, 0, 0))
                self.science_temperature_plot.setLabel("left", "Temperature")
                self.science_temperature_plot.setLabel("bottom", "Time")
                self.science_temperature_plot.plot(self.temp_plot_data[1], self.temp_plot_data[0], 
                                                 pen=pen, symbol="+", symbolSize=10, symbolBrush="b")
            
            # Update PMT plot
            if self.pmt_plot_data != []:
                self.science_pmt_plot.show()  # Show the plot
                self.science_pmt_plot.clear()
                pen = pg.mkPen(color=(0, 255, 0))
                self.science_pmt_plot.setLabel("left", "PMT")
                self.science_pmt_plot.setLabel("bottom", "Time")
                self.science_pmt_plot.plot(self.pmt_plot_data[1], self.pmt_plot_data[0], 
                                          pen=pen, symbol="+", symbolSize=10, symbolBrush="r")


class SensorBlock(QWidget):
    def __init__(self, ui: RoverGUI):
        super().__init__()
        self.ui = ui
        self.setMinimumSize(100, 40)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Change to Minimum
        
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        
        # Make labels more visible
        probe_label = QLabel("Probe")
        probe_label.setStyleSheet("font-weight: bold;")
        fad_label = QLabel("FAD")
        fad_label.setStyleSheet("font-weight: bold;")
        
        self.read_button = QPushButton("Read")
        self.save_button = QPushButton("Save")
        self.data_button = QPushButton("Data")
        self.fad_main_button = QPushButton("Main")
        self.fad_button_1 = QPushButton("Site 1")
        self.fad_button_2 = QPushButton("Site 2")
        self.read_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.save_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.data_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.fad_main_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.fad_button_1.setStyleSheet("padding: 4px; min-height: 20px;")
        self.fad_button_2.setStyleSheet("padding: 4px; min-height: 20px;")
        self.read_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.save_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.data_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fad_main_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fad_button_1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fad_button_2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.read_button.clicked.connect(self.read)
        self.save_button.clicked.connect(self.save)
        self.data_button.clicked.connect(self.data)
        self.fad_main_button.clicked.connect(self.fad_main)
        self.fad_button_1.clicked.connect(self.fad_1)
        self.fad_button_2.clicked.connect(self.fad_2)
        
        self.layout.addWidget(probe_label)
        self.layout.addWidget(self.read_button)
        self.layout.addWidget(self.save_button)
        self.layout.addWidget(self.data_button)
        self.layout.addWidget(fad_label)
        self.layout.addWidget(self.fad_main_button)
        self.layout.addWidget(self.fad_button_1)
        self.layout.addWidget(self.fad_button_2)
        
        self.setLayout(self.layout)
        self.setFixedHeight(40)
        self.reading = False
        self.start_read = False
        self.stop_read = False
        self.display = False

    def read(self):
        print("Read Sensor")
        self.reading = not self.reading
        self.start_read = self.reading
        if self.reading:
            self.read_button.setStyleSheet("background-color: #FF0000;")
        else:
            self.read_button.setStyleSheet("background-color: #FFFFFF;")
            self.stop_read = True
    
    def save(self):
        """Save temperature and humidity data to CSV and graphs to PNG"""
        print("Saving sensor data...")
        
        # Create directory if it doesn't exist
        csv_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data")
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        
        # Create a timestamp for unique filenames
        timestr = time.strftime("%Y%m%d-%H%M%S")
        saved_something = False
        
        # Save temperature data to CSV if available
        if self.ui.temp_save_data:
            # Use saved data
            temp_csv_path = os.path.join(csv_dir, f"temperature_data_{timestr}.csv")
            with open(temp_csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Temperature (°C)"])
                for i in range(len(self.ui.temp_save_data[0])):
                    writer.writerow([self.ui.temp_save_data[1][i], self.ui.temp_save_data[0][i]])
        
            # Save temperature graph as PNG using matplotlib
            temp_png_path = os.path.join(csv_dir, f"temperature_graph_{timestr}.png")
            plt.figure(figsize=(10, 6))
            plt.plot(self.ui.temp_save_data[1][1:], self.ui.temp_save_data[0][1:], 'r-+', markersize=8)
            plt.grid(True, alpha=0.3)
            plt.title("Temperature Data")
            plt.xlabel("Time")
            plt.ylabel("Temperature (°C)")
            plt.savefig(temp_png_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            print(f"Temperature data saved to {temp_csv_path}")
            print(f"Temperature graph saved to {temp_png_path}")
            saved_something = True
            
            # Clear buffers after saving
            self.ui.temp_data_buffer.clear()
            self.ui.temp_time_buffer.clear()
    
        # Save humidity data to CSV if available
        if self.ui.hum_save_data:
            # Use saved data instead of plot data
            hum_csv_path = os.path.join(csv_dir, f"humidity_data_{timestr}.csv")
            with open(hum_csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Humidity (%)"])
                for i in range(len(self.ui.hum_save_data[0])):
                    writer.writerow([self.ui.hum_save_data[1][i], self.ui.hum_save_data[0][i]])
        
            # Save humidity graph as PNG using matplotlib
            hum_png_path = os.path.join(csv_dir, f"humidity_graph_{timestr}.png")
            plt.figure(figsize=(10, 6))
            plt.plot(self.ui.hum_save_data[1][1:], self.ui.hum_save_data[0][1:], 'b-+', markersize=8)
            plt.grid(True, alpha=0.3)
            plt.title("Humidity Data")
            plt.xlabel("Time")
            plt.ylabel("Humidity (%)")
            plt.savefig(hum_png_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            print(f"Humidity data saved to {hum_csv_path}")
            print(f"Humidity graph saved to {hum_png_path}")
            saved_something = True
            
            # Clear buffers after saving
            self.ui.hum_data_buffer.clear()
            self.ui.hum_time_buffer.clear() 
    
        # Show a confirmation message
        if saved_something:
            # Flash the button to indicate successful save
            original_style = self.save_button.styleSheet()
            self.save_button.setStyleSheet("background-color: #00FF00; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.save_button.setStyleSheet(original_style))
        else:
            # Flash the button red to indicate no data to save
            original_style = self.save_button.styleSheet()
            self.save_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.save_button.setStyleSheet(original_style))
            print("No data available to save")

    def data(self):
        print("Data Sensor")
        self.display = not self.display
        if self.display:
            self.ui.science_temperature_plot.show()  # Show the plot
            self.ui.science_humidity_plot.show()  # Show the plot
            self.data_button.setStyleSheet("background-color: #FF0000;")
        else:
            self.ui.science_temperature_plot.hide()  # Hide the plot
            self.ui.science_humidity_plot.hide()  # Hide the plot
            self.data_button.setStyleSheet("background-color: #FFFFFF;")
    
    def fad_main(self):
        self.ui.science_serial_controller.publish("<Y>")
        print("FAD Main")

    def fad_1(self):
        self.ui.science_serial_controller.publish("<X>")
        print("FAD 1")
    
    def fad_2(self):
        self.ui.science_serial_controller.publish("<Z>")
        print("FAD 2")
        

class SampleBlock(QWidget):
    def __init__(self, chem: bool, ui: RoverGUI):
        super().__init__()
        self.ui = ui
        self.setMinimumSize(100, 100)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  # Change to Expanding
        self.site1_block = SampleSubBlock(chem, True, ui)
        if chem:
            self.site2_block = SampleSubBlock(chem, False, ui)

        self.chem = chem

        self.top_bar_layout = QHBoxLayout()
        if chem:
            self.top_bar_layout.addWidget(QLabel("Chem"))
            self.mid_button = QPushButton("Ninhydrin")
        else:
            self.pmtWidget = QLabel("PMT")
            self.top_bar_layout.addWidget(self.pmtWidget)
            self.mid_button = QPushButton("Switch")
        self.data_button = QPushButton("Data")
        
        # Set expanding size policy for the buttons
        self.mid_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.data_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.top_bar_layout.addWidget(self.mid_button)
        self.top_bar_layout.addWidget(self.data_button)
        self.top_bar_layout.setContentsMargins(0, 0, 0, 0)
        self.top_bar_layout.setSpacing(0)
        self.mid_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.data_button.setStyleSheet("padding: 4px; min-height: 20px;")
        self.mid_button.clicked.connect(self.switch)
        self.data_button.clicked.connect(self.data)
        self.top_bar = QWidget()
        self.top_bar.setLayout(self.top_bar_layout)
        self.top_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Change to Expanding

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.top_bar)
        self.layout.addWidget(self.site1_block)
        if chem:
            self.layout.addWidget(self.site2_block)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        self.setLayout(self.layout)

        self.display = False

    def switch(self):
        if self.chem:
            self.ui.science_serial_controller.publish("<N>")
            print("Ninhydrin")
        else:
            self.ui.science_serial_controller.publish("<S>")
            print("Switch PMT")
    
    def data(self):
        self.display = not self.display
        if self.chem:
            print("Data Chem")
        else:
            print("Data Soil")
        
        if self.display:
            self.data_button.setStyleSheet("background-color: #FF0000;")
            if self.chem:
                self.ui.science_ph1_plot.show()
                self.ui.science_ph2_plot.show()
            else:
                self.ui.science_pmt_plot.show()
        else:
            self.data_button.setStyleSheet("background-color: #FFFFFF;")
            if self.chem:
                self.ui.science_ph1_plot.hide()
                self.ui.science_ph2_plot.hide()
            else:
                self.ui.science_pmt_plot.hide()

class SampleSubBlock(QWidget):
    def __init__(self, chem: bool, site1: bool, ui: RoverGUI):
        super().__init__()
        self.ui = ui
        self.setMinimumSize(100, 40)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  # Change to Expanding
        
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        
        self.chem = chem
        self.site1 = site1
        
        if not chem:
            self.smaple_button = QPushButton("Sample")
            self.read_button = QPushButton("Read")
            self.save_button = QPushButton("Save")
        elif self.site1:
            self.smaple_button = QPushButton("Site 1 Sample")
            self.read_button = QPushButton("Read Site 1")
            self.save_button = QPushButton("Save Site 1")
        else:
            self.smaple_button = QPushButton("Site 2 Sample")
            self.read_button = QPushButton("Read Site 2")
            self.save_button = QPushButton("Save Site 2")
            
        buttonStyle = "padding: 4px; min-height: 20px;"
        self.smaple_button.setStyleSheet(buttonStyle)
        self.read_button.setStyleSheet(buttonStyle)
        self.save_button.setStyleSheet(buttonStyle)
        
        # Set all buttons to use expanding size policy
        self.smaple_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.read_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.save_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.smaple_button.clicked.connect(self.sample)
        self.read_button.clicked.connect(self.read)
        self.save_button.clicked.connect(self.save)

        # Use a horizontal layout with a 2:1 ratio between sample button and right side
        hLayout = QHBoxLayout()
        hLayout.setContentsMargins(0, 0, 0, 0)
        hLayout.setSpacing(0)
        
        # Create vertical layout for read/save buttons
        rightVLayout = QVBoxLayout()
        rightVLayout.addWidget(self.read_button)
        rightVLayout.addWidget(self.save_button)
        rightVLayout.setContentsMargins(0, 0, 0, 0)
        rightVLayout.setSpacing(0)
        
        rightWidget = QWidget()
        rightWidget.setLayout(rightVLayout)
        rightWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Add widgets with stretch factors to control relative sizes
        hLayout.addWidget(self.smaple_button, 2)  # 2/3 of space
        hLayout.addWidget(rightWidget, 1)        # 1/3 of space
        
        self.layout.addLayout(hLayout)
        self.setLayout(self.layout)

        self.reading = False
        self.start_read = False
        self.stop_read = False

    def sample(self):
        if self.chem:
            if self.site1:
                self.ui.science_serial_controller.publish("<A>")
                print("Sampled Chem Site 1")
            else:
                self.ui.science_serial_controller.publish("<B>")
                print("Sampled Chem Site 2")
        else:
            if self.ui.pmt_switch == 1:
                self.ui.science_serial_controller.publish("<D>")
                print("Sampled FAM Site 1")
            elif self.ui.pmt_switch == 2:
                self.ui.science_serial_controller.publish("<E>")
                print("Sampled FAM Site 2")
    
    def read(self):
        self.reading = not self.reading
        self.start_read = self.reading
        if not self.reading:
            self.stop_read = True
        if self.chem:
            print("Read Chem")
        else:
            print("Read FAM")

        if self.reading:
            if self.chem:
                self.read_button.setText("Stop Reading Site 1" if self.site1 else "Stop Reading Site 2")
            else:
                self.read_button.setText("Stop Reading")
            self.read_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
        else:
            if self.chem:
                self.read_button.setText("Read Site 1" if self.site1 else "Read Site 2")
            else:
                self.read_button.setText("Read")
            self.read_button.setStyleSheet("background-color: #FFFFFF; padding: 4px; min-height: 20px;")

    def save(self):
        """Save sample data to CSV and graphs to PNG"""
        print(f"Saving {'chemistry' if self.chem else 'FAM'} data...")
        
        # Create directory if it doesn't exist
        csv_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data")
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        
        # Create a timestamp for unique filenames
        timestr = time.strftime("%Y%m%d-%H%M%S")
        
        # Determine which data to save based on chemistry/FAM and site
        if self.chem:
            if self.site1:
                data = self.ui.ph1_save_data  # Use saved data
                data_type = "ph1"
                title = "Site 1 pH"
                color = 'r'
            else:
                data = self.ui.ph2_save_data  # Use saved data
                data_type = "ph2"
                title = "Site 2 pH"
                color = 'r'
        else:
            # FAM/Soil data (PMT)
            data = self.ui.pmt_save_data  # Use saved data
            data_type = "pmt"
            title = f"Site {self.ui.pmt_switch} PMT"
            color = 'g'
        
        # Save data to CSV if available
        if data and len(data[0]) > 0:
            # Save data to CSV
            csv_path = os.path.join(csv_dir, f"{data_type}_data_{timestr}.csv")
            with open(csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", f"{title} Value"])
                for i in range(len(data[0])):
                    writer.writerow([data[1][i], data[0][i]])
            
            # Save graph as PNG using matplotlib
            png_path = os.path.join(csv_dir, f"{data_type}_graph_{timestr}.png")
            plt.figure(figsize=(10, 6))
            plt.plot(data[1][1:], data[0][1:], f'{color}-+', markersize=8)
            plt.grid(True, alpha=0.3)
            plt.title(f"{title} Data")
            plt.xlabel("Time")
            plt.ylabel(title)
            plt.savefig(png_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            print(f"{title} data saved to {csv_path}")
            print(f"{title} graph saved to {png_path}")
            
            # Clear buffers after saving
            if self.chem:
                if self.site1:
                    self.ui.ph1_data_buffer.clear()
                    self.ui.ph1_time_buffer.clear()
                else:
                    self.ui.ph2_data_buffer.clear()
                    self.ui.ph2_time_buffer.clear()
            else:
                self.ui.pmt_data_buffer.clear()
                self.ui.pmt_time_buffer.clear()
                
            # Give visual feedback - briefly change button color to green
            original_style = self.save_button.styleSheet()
            self.save_button.setStyleSheet("background-color: #00FF00; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.save_button.setStyleSheet(original_style))
        else:
            # No data to save - briefly change button color to red
            original_style = self.save_button.styleSheet()
            self.save_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.save_button.setStyleSheet(original_style))
            print(f"No {title} data available to save")

class GenieControl(QWidget):
    def __init__(self, node, ui: RoverGUI):
        super().__init__()
        self.node = node
        # Create a proper layout
        self.layout = QVBoxLayout()
        
        self.ui = ui

        # Create a row layout for buttons
        button_layout = QHBoxLayout()

        microscope_layout = QVBoxLayout()
        
        # Create buttons
        self.take_pano_button = QPushButton("Take Pano")
        self.toggle_genie_button = QPushButton("Toggle Genie Filter")
        self.save_genie_button = QPushButton("Save Genie Image")  # New button
        self.microscope_zoom_in_button = QPushButton("+")
        self.microscope_zoom_out_button = QPushButton("-")
        self.micro_slider_splitter = Slider(Qt.Horizontal)
        
        # Set button styles
        button_style = "padding: 4px; min-height: 20px;"
        self.take_pano_button.setStyleSheet(button_style)
        self.toggle_genie_button.setStyleSheet(button_style)
        self.save_genie_button.setStyleSheet(button_style)  # Style for new button
        self.microscope_zoom_in_button.setStyleSheet(button_style)
        self.microscope_zoom_out_button.setStyleSheet(button_style)
        self.micro_slider_splitter.setMinimum(0)
        self.micro_slider_splitter.setMaximum(100)
        self.micro_slider_splitter.setValue(50)

        # Set size policies to make buttons fit their content
        self.take_pano_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.toggle_genie_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.save_genie_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)  # Size policy for new button
        self.microscope_zoom_in_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.microscope_zoom_out_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        
        # Set fixed dimensions for zoom + slider 
        self.microscope_zoom_in_button.setFixedSize(30, 30)
        self.microscope_zoom_out_button.setFixedSize(30, 30)
        self.micro_slider_splitter.setFixedSize(500, 30)
        self.micro_slider_splitter.valueUpdated.connect(self.change_microscope_zoom_magnitude)

        # Connect button signals to slots
        self.take_pano_button.clicked.connect(self.take_pano)
        self.toggle_genie_button.clicked.connect(self.toggle_genie)
        self.save_genie_button.clicked.connect(self.save_genie_image)  # Connect new button
        self.microscope_zoom_in_button.clicked.connect(self.zoom_in_microscope)
        self.microscope_zoom_out_button.clicked.connect(self.zoom_out_microscope)

        # Add zoom buttons to vertical layout
        microscope_layout.addWidget(self.microscope_zoom_in_button)
        microscope_layout.addWidget(self.microscope_zoom_out_button)
        microscope_layout.setSpacing(2)
        
        # Create a widget for the microscope controls
        microscope_widget = QWidget()
        microscope_widget.setLayout(microscope_layout)
        microscope_widget.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)

        self.site1checkbox = QCheckBox("Site 1")
        self.site2checkbox = QCheckBox("Site 2")
        self.site1checkbox.setChecked(False)
        self.site2checkbox.setChecked(False)
        self.genie_site1 = False
        self.genie_site2 = False

        self.pano_site1 = False
        self.pano_site2 = False
        self.site1checkbox.stateChanged.connect(self.site1checkbox_changed)
        self.site2checkbox.stateChanged.connect(self.site2checkbox_changed)
        
        # Add stretch FIRST to push buttons to the right
        button_layout.addStretch(1)

        self.wavelength_display = QLabel("Wavelength: None")
        self.wavelengths = [None, 440, 500, 530, 570, 610, 670, 740, 780, 840, 900, 950, 1000]
        self.curr_l = None
        self.curr_l_count = 0

        # Add buttons to the row layout (now they'll be right-aligned)
        button_layout.addWidget(self.take_pano_button)
        # button_layout.addWidget(QLabel("Site 1"))
        button_layout.addWidget(self.site1checkbox)
        # button_layout.addWidget(QLabel("Site 2"))
        button_layout.addWidget(self.site2checkbox)
        button_layout.addWidget(self.wavelength_display)
        button_layout.addWidget(self.toggle_genie_button)
        button_layout.addWidget(self.save_genie_button)  # Add new button
        button_layout.addWidget(self.micro_slider_splitter)
        button_layout.addWidget(microscope_widget)
        
        # Add the button row to the main layout
        self.layout.addLayout(button_layout)
        
        # Apply the layout to the widget
        self.setLayout(self.layout)

        self.zoom_mag = 5


        self.pano_control = node.create_publisher(Bool, '/pano_control', 10)
        self.pano_result = node.create_subscription(Image, '/pano_result', self.pano_callback, 10)
        self.pano_img = node.create_subscription(Image, '/pano_img', self.pano_individual_callback, 10)
        self.get_genie_pub = node.create_publisher(String,  '/save_genie_image', 10)
        
        
        self.feed = ui.camera_feed

        self.show_pano_button(False)
        # self.show_genie_button(False)
        self.show_zoom_controls(False)
        self.show_save_genie_button(True)  # Initially hide save button

    def site1checkbox_changed(self, state):
        if state == Qt.Checked:
            self.genie_site1 = True
            self.site1 = True
            print("Site 1 selected")
        else:
            self.genie_site1 = False
            self.pano_site1 = False
            print("Site 1 deselected")
    
    def site2checkbox_changed(self, state):
        if state == Qt.Checked:
            self.genie_site2 = True
            self.pano_site2 = True
            print("Site 2 selected")
        else:
            self.genie_site2 = False
            self.pano_site2 = False
            print("Site 2 deselected")

    def show_pano_button(self, show=True):
        """Show or hide the panorama button"""
        self.take_pano_button.setVisible(show)
    
    def show_genie_button(self, show=True):
        """Show or hide the genie filter toggle button"""
        self.toggle_genie_button.setVisible(show)
    
    def show_zoom_controls(self, show=True):
        """Show or hide both zoom buttons"""
        self.micro_slider_splitter.setVisible(show)
        self.microscope_zoom_in_button.setVisible(show)
        self.microscope_zoom_out_button.setVisible(show)
    
    def show_save_genie_button(self, show=True):
        """Show or hide the save genie image button"""
        self.save_genie_button.setVisible(show)
    
    def take_pano(self):
        # Implement the logic to take a panorama
        print("Taking panorama...")
        self.pano_control.publish(True)

    def toggle_genie_lambda(self):
        self.toggle_genie_button.setStyleSheet(self.genie_button_style)
        self.curr_l_count += 1
        if self.curr_l_count >= len(self.wavelengths):
            self.curr_l_count = 0
        self.curr_l = self.wavelengths[self.curr_l_count]
        self.wavelength_display.setText("Wavelength: " + str(self.curr_l) + "nm" if self.curr_l is not None else "Wavelength: None")
        self.toggle_genie_button.setEnabled(True)
    
    def toggle_genie(self):
        # Implement the logic to toggle the genie lens
        self.ui.science_serial_controller.publish("<M>")
        print("Toggling genie filter...")
        self.genie_button_style = self.toggle_genie_button.styleSheet()
        self.toggle_genie_button.setEnabled(False)
        self.toggle_genie_button.setStyleSheet("background-color: #00FF00; padding: 4px; min-height: 20px;")
        QTimer.singleShot(5000, self.toggle_genie_lambda)

    def zoom_in_microscope(self):
        # Implement the logic to zoom in on the microscope
        self.ui.science_serial_controller.publish("<I," + str(self.zoom_mag) + ">")
        print("Zooming in on microscope...")

    def zoom_out_microscope(self):
        # Implement the logic to zoom out on the microscope
        self.ui.science_serial_controller.publish("<O," + str(self.zoom_mag) + ">")
        print("Zooming out on microscope...")

    def change_microscope_zoom_magnitude(self, value):
        self.zoom_mag = value // 10

    def pano_callback(self, msg):
        """Save the panorama image when received"""
        try:
            # Create bridge to convert ROS Image to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Create directory if it doesn't exist
            pano_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data", "panoramas")
            if not os.path.exists(pano_dir):
                os.makedirs(pano_dir)
            
            # Create a timestamp for a unique filename
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(pano_dir, f"panorama_{timestr}.png")
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Notify users
            print(f"Panorama saved to {filename}")
            
        except Exception as e:
            print(f"Error saving panorama: {e}")
            # Flash the button red to indicate failure
            original_style = self.take_pano_button.styleSheet()
            self.take_pano_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.take_pano_button.setStyleSheet(original_style))
    
    def pano_individual_callback(self, msg):
        try:
            # Create bridge to convert ROS Image to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Create directory if it doesn't exist
            """Save the current genie camera image"""
            if self.pano_site1 and self.pano_site2:
                site = "Site_3"
            elif self.pano_site1:
                site = "Site_1"
            elif self.pano_site2:
                site = "Site_2"
            else:
                site = "Site_0"

            pano_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data", "panoramas_individual")
            if not os.path.exists(pano_dir):
                os.makedirs(pano_dir)
            
            # Create a timestamp for a unique filename
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(pano_dir, f"panorama_{site}_{timestr}.png")
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Notify user
            print(f"Panoramas saved to {filename}")
            
        except Exception as e:
            print(f"Error saving panoramas: {e}")
            # Flash the button red to indicate failure
            original_style = self.take_pano_button.styleSheet()
            self.take_pano_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.take_pano_button.setStyleSheet(original_style))


    def save_genie_image(self):
        """Save the current genie camera image"""
        if self.genie_site1 and self.genie_site2:
            site = "Site_3"
        elif self.genie_site1:
            site = "Site_1"
        elif self.genie_site2:
            site = "Site_2"
        else:
            site = "Site_0"
        self.get_genie_pub.publish(site + "," + (str(self.curr_l) + "nm" if self.curr_l is not None else "None"))
        print("Saving genie image...")

    # def genie_image_callback(self, msg):
    #     """Store the latest genie camera image"""
    #     self.last_genie_image = msg
        

class CheckableComboBox(QComboBox):
    def __init__(self, title = '', parent=None):
        super().__init__(parent)
        self.setTitle(title)
        self.view().pressed.connect(self.handleItemPressed)
        self.setModel(QStandardItemModel())

    def handleItemPressed(self, index):
        item = self.model().itemFromIndex(index)
        if item.checkState() == Qt.Checked:
            item.setCheckState(Qt.Unchecked)
        else:
            item.setCheckState(Qt.Checked)

    def title(self):
        return self._title

    def setTitle(self, title):
        self._title = title
        self.repaint()

    def paintEvent(self, event):
        painter = QStylePainter(self)
        painter.setPen(self.palette().color(QPalette.Text))
        opt = QStyleOptionComboBox()
        self.initStyleOption(opt)
        opt.currentText = self._title
        painter.drawComplexControl(QStyle.CC_ComboBox, opt)
        painter.drawControl(QStyle.CE_ComboBoxLabel, opt)

class CameraSelect(QWidget):
    cameras_changed = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.layout = QHBoxLayout()  # Change to HBoxLayout for better horizontal layout
        self.layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins
        
        self.toolButton = QToolButton(self)
        self.toolButton.setText("Cameras List")
        self.toolMenu = QMenu(self)

        self.cameras = ["Zed (front) camera", "Butt camera", "Genie camera", "Microscope camera", "Webcam"]
        self.selected_cameras = {camera: False for camera in self.cameras}

        for camera in self.cameras:
            action = self.toolMenu.addAction(camera)
            action.setCheckable(True)
            self.toolMenu.triggered.connect(self.handle_camera_selection)

        self.toolButton.setMenu(self.toolMenu)
        self.toolButton.setPopupMode(QToolButton.InstantPopup)
        self.layout.addWidget(self.toolButton)
        self.layout.addStretch(1)
        self.setLayout(self.layout)
        
        # Set a maximum height for the widget
        self.setMaximumHeight(40)

    def handle_camera_selection(self, action):
        camera = action.text()
        self.selected_cameras[camera] = action.isChecked()
        self.cameras_changed.emit(self.selected_cameras)


if __name__ == '__main__':
    rclpy.init()
    node=rclpy.create_node('rover_gui')
    # rospy.init_node('rover_gui', anonymous=False)
    app = QApplication(sys.argv)
    gui = RoverGUI(node)

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