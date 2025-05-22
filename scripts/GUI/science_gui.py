#!/usr/bin/env python3


import csv
import sys
import rospy
import map_viewer as map_viewer
from pathlib import Path
import numpy as np
import os
import time


from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, \
    QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame, \
    QCheckBox,QSplitter,QStylePainter, QStyleOptionComboBox, QStyle, \
    QToolButton, QMenu, QLineEdit , QPushButton, QTextEdit,\
    QListWidget, QListWidgetItem, QStyleOptionSlider, QSizePolicy
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from geometry_msgs.msg import Twist
# from rover.arm.ros1.gripper import arm_serial_connector
from sensor_msgs.msg import NavSatFix, CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String, Bool
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter,QPalette,QStandardItemModel, QTextCursor, QFont
from calian_gnss_ros2_msg.msg import GnssSignalStatus
import pyqtgraph as pg



#cache folder of map tiles generated from tile_scraper.py
CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"
science_arduino_board_name = None #Glob variable for board name, SHOULD BE CHANGED

#map widget that has map viewer 
class mapOverlay(QWidget):
    def __init__(self):
        super().__init__()
        
        self.viewer = map_viewer.MapViewer()
        #sets the source of map tiles to local tile cache folder
        self.viewer.set_map_server(
            str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
        )
        self.setLayout(self.initOverlayout())
        self.centreOnRover = False
        
        # ROS Subscriber for GPS coordinates
        rospy.Subscriber('/calian_gnss/gps', NavSatFix, self.update_gps_coordinates)
        rospy.Subscriber('/calian_gnss/gps_extended', GnssSignalStatus, self.update_gps_heading)
    
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
            self.viewer.center_on_gps( gps_point) 

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

    def __init__(self):
        super().__init__()
        self.init_ui()

        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        rospy.Subscriber('aruco_found', Bool, self.bool_callback)
        rospy.Subscriber('aruco_name', String, self.string_callback)

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

    def __init__(self):
        super().__init__()
        self.init_ui()

        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        rospy.Subscriber('aruco_found', Bool, self.bool_callback)
        rospy.Subscriber('aruco_name', String, self.string_callback)

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

    def __init__(self):
        super().__init__()
        self.init_ui()

        # Connect signals to the corresponding update methods
        self.update_mallet_signal.connect(self.update_mallet)
        self.update_bottle_signal.connect(self.update_bottle)

        # Initialize ROS subscribers
        rospy.Subscriber('mallet_detected', Bool, self.mallet_callback)
        rospy.Subscriber('waterbottle_detected', Bool, self.bottle_callback)

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

    def __init__(self):
        super().__init__()
        self.init_ui()

        # Connect the signal to the update method
        self.update_label_signal.connect(self.update_label)

        # Initialize ROS subscriber
        rospy.Subscriber('/led_colour', String, self.callback)

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
    def __init__(self, map_overlay):
        super().__init__()
        self.longLat_pub = rospy.Publisher('/long_lat_goal_array', Float32MultiArray, queue_size=5)
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
    def __init__(self, map_overlay):
        super().__init__()
        self.longLat_pub = rospy.Publisher('/long_lat_goal_array', Float32MultiArray, queue_size=5)
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
    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=10)
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


class CameraFeed:
    def __init__(self, label1, label2, label3, label4, label5, splitter):
        self.active_cameras = {"Zed (front) camera": False, "Butt camera": False, "Microscope camera": False, "Genie camera": False, "Webcam": False}
        self.bridge = CvBridge()
        self.image_sub1 = None
        self.image_sub2 = None
        self.image_sub3 = None
        self.image_sub4 = None
        self.image_sub5 = None
        self.state_sub = rospy.Subscriber("state", String, self.state_callback)

        self.obj_bbox = rospy.Subscriber("object/bbox", Float64MultiArray, self.bbox_callback)
        self.bbox_sub = rospy.Subscriber("aruco_node/bbox", Float64MultiArray, self.bbox_callback)

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

    def register_subscriber1(self):
        if self.image_sub1 is None:
            self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color/compressed", CompressedImage, self.callback1)

    def unregister_subscriber1(self):
        if self.image_sub1:
            self.image_sub1.unregister()
            self.image_sub1 = None

    def register_subscriber2(self):
        if self.image_sub2 is None:
            self.image_sub2 = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback2)

    def unregister_subscriber2(self):
        if self.image_sub2:
            self.image_sub2.unregister()
            self.image_sub2 = None

    def register_subscriber3(self):
        if self.image_sub3 is None:
            self.image_sub3 = rospy.Subscriber("/microscope", Image, self.callback3)

    def unregister_subscriber3(self):
        if self.image_sub3:
            self.image_sub3.unregister()
            self.image_sub3 = None
        
    def register_subscriber4(self):
        if self.image_sub4 is None:
            self.image_sub4 = rospy.Subscriber("/geniecam", Image, self.callback4)
    
    def unregister_subscriber4(self):
        if self.image_sub4:
            self.image_sub4.unregister()
            self.image_sub4 = None

    def register_subscriber5(self):
        if self.image_sub5 is None:
            self.image_sub5 = rospy.Subscriber("/webcam/compressed", CompressedImage, self.callback5)

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
            self.update_microscope_image(data, self.label3)

    def callback4(self, data):
        if self.active_cameras["Genie camera"]:
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

#main gui class, make updates here to change top level hierarchy
class RoverGUI(QMainWindow):
    statusSignal = pyqtSignal(str)
    probeUpdateSignal = pyqtSignal(bool)
    def __init__(self):
        super().__init__()
        self.statusTerminal = statusTerminal()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.velocity_control = VelocityControl()
        self.gui_status_sub = rospy.Subscriber('gui_status', String, self.string_callback)
        # self.auto_abort_pub = rospy.Publisher('/auto_abort_check', Bool, queue_size=5)
        # self.next_state_pub = rospy.Publisher('/next_state', Bool, queue_size=5)
        self.science_serial_controller = rospy.Publisher('/science_serial_control', String, queue_size=5)
        self.science_serial_data = rospy.Subscriber('/science_serial_data', String, self.get_probe_data_callback)
        self.reached_state = None
        self.ph1_plot_data = []
        self.ph2_plot_data = []
        self.hum_plot_data = []
        self.temp_plot_data = []
        self.pmt_plot_data = []
        self.ph1_time, self.ph2_time, self.hum_time, self.temp_time, self.pmt_time = (1, 1, 1, 1, 1)

        # Create tabs
        self.scienceTab = QWidget()  # Create science tab first
        self.longlat_tab = QWidget()
        self.controlTab = QWidget()
        self.camsTab = QWidget()

        # Setup tabs before adding them
        self.setup_science_tab()  # Setup science tab first
        self.setup_lngLat_tab()
        self.setup_control_tab()
        self.setup_cams_tab()
        
        # Add tabs to QTabWidget - with science tab first
        self.tabs.addTab(self.scienceTab, "Science")  # Science tab is now first/default
        self.tabs.addTab(self.longlat_tab, "State Machine")
        self.tabs.addTab(self.controlTab, "Controls")
        self.tabs.addTab(self.camsTab, "Cameras")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)
        
        self.statusSignal.connect(self.string_signal_receive)
        self.probeUpdateSignal.connect(self.update_science_plot)

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
        


        
    #unused utility: if multiple tabs used can have triggers when tab sswitched
    def on_tab_changed(self, index):
        if index == 1:  # Map Tab
            print("map tab")  
        elif index == 2:  # Split Screen Tab
            print("split tab") # Show map viewer in split screen tab


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
        self.camera_feed_cams_tab = CameraFeed(self.camera_label1_cams_tab, self.camera_label2_cams_tab, self.camera_label3_cams_tab, self.camera_label4_cams_tab, self.camera_label5_cams_tab, self.camerasplitter_cams_tab)
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
        self.lngLatEntry = LngLatEntryBar(self.map_overlay)
        self.lngLatFile = LngLatEntryFromFile(self.map_overlay)
        self.lngLatDeliveryFile = LngLatDeliveryEntryFromFile(self.map_overlay)
        self.stateMachineDialog = StateMachineStatus()
        self.arucoBox = ArucoWidget()
        

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
        self.map_overlay = mapOverlay()
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
        self.camera_feed = CameraFeed(self.camera_label1, self.camera_label2, self.camera_label3, self.camera_label4, self.camera_label5, self.camerasplitter)
        self.camerasplitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.select_splitter = QSplitter(Qt.Horizontal)
        self.select_splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Change to Minimum
        self.select_splitter.setMaximumHeight(50)  # Set a maximum height

        # Use CameraSelect menu-based selector
        self.camera_selector = CameraSelect()
        self.camera_selector.cameras_changed.connect(self.update_active_cameras_science)
        self.camera_selector.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)  # Ensure this is minimal height

        self.genieControl = GenieControl(self)
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
        msgs = data.data.split(";")
        msg = []
        for i in msgs:
            if i == "":
                continue
            msg.append(i.split(":"))

        # PH1 block (no changes needed - this is already correct)
        if self.left_middle_item1.display or self.left_middle_item1.site1_block.reading:
            if self.ph1_time == 1:
                ph1_time_buffer = []
                ph1_data_buffer = []
            ph1_time_buffer.append(self.ph1_time)
            ph1_graph_data = float(msg[0][1])
            ph1_data_buffer.append(ph1_graph_data)
            self.ph1_time += 1
        if self.left_middle_item1.site1_block.start_read:
            self.ph1_start = len(ph1_data_buffer)
            self.left_middle_item1.site1_block.start_read = False
        if not (self.left_middle_item1.site1_block.start_read or self.left_middle_item1.site1_block.reading):
            self.ph1_plot_data = [ph1_data_buffer[self.ph1_start:-1], ph1_time_buffer[self.ph1_start:-1]]
            self.ph1_plot_avg = sum(self.ph1_plot_data[0]) / len(self.ph1_plot_data[0])
            self.ph1_plot_data[0].insert(0, self.ph1_plot_avg)
            self.ph1_plot_data[1].insert(0, -1)
            self.ph1_time = 1
            ph1_data_buffer.clear()
            ph1_time_buffer.clear()

        # PH2 block - updated to match ph1 pattern
        if self.left_middle_item1.display or self.left_middle_item1.site2_block.reading:
            if self.ph2_time == 1:
                ph2_time_buffer = []
                ph2_data_buffer = []
            ph2_time_buffer.append(self.ph2_time)
            ph2_graph_data = float(msg[1][1])
            ph2_data_buffer.append(ph2_graph_data)
            self.ph2_time += 1
        if self.left_middle_item1.site2_block.start_read:
            self.ph2_start = len(ph2_data_buffer)
            self.left_middle_item1.site2_block.start_read = False
        if not (self.left_middle_item1.site2_block.start_read or self.left_middle_item1.site2_block.reading):
            self.ph2_plot_data = [ph2_data_buffer[self.ph2_start:-1], ph2_time_buffer[self.ph2_start:-1]]
            self.ph2_plot_avg = sum(self.ph2_plot_data[0]) / len(self.ph2_plot_data[0])
            self.ph2_plot_data[0].insert(0, self.ph2_plot_avg)
            self.ph2_plot_data[1].insert(0, -1)
            self.ph2_time = 1
            ph2_data_buffer.clear()
            ph2_time_buffer.clear()
        
        # HUM block - updated to match ph1 pattern
        if self.left_top_widget.display or self.left_top_widget.site1_block.reading:
            if self.hum_time == 1:
                hum_time_buffer = []
                hum_data_buffer = []
            hum_time_buffer.append(self.hum_time)
            hum_graph_data = float(msg[2][1])
            hum_data_buffer.append(hum_graph_data)
            self.hum_time += 1
        if self.left_top_widget.site1_block.start_read:
            self.hum_start = len(hum_data_buffer)
            self.left_top_widget.site1_block.start_read = False
        if not (self.left_top_widget.site1_block.start_read or self.left_top_widget.site1_block.reading):
            self.hum_plot_data = [hum_data_buffer[self.hum_start:-1], hum_time_buffer[self.hum_start:-1]]
            self.hum_plot_avg = sum(self.hum_plot_data[0]) / len(self.hum_plot_data[0])
            self.hum_plot_data[0].insert(0, self.hum_plot_avg)
            self.hum_plot_data[1].insert(0, -1)
            self.hum_time = 1
            hum_data_buffer.clear()
            hum_time_buffer.clear()
        
        # TEMP block - updated to match ph1 pattern
        if self.left_top_widget.display or self.left_top_widget.site2_block.reading:
            if self.temp_time == 1:
                temp_time_buffer = []
                temp_data_buffer = []
            temp_time_buffer.append(self.temp_time)
            temp_graph_data = float(msg[3][1])
            temp_data_buffer.append(temp_graph_data)
            self.temp_time += 1
        if self.left_top_widget.site2_block.start_read:
            self.temp_start = len(temp_data_buffer)
            self.left_top_widget.site2_block.start_read = False
        if not (self.left_top_widget.site2_block.start_read or self.left_top_widget.site2_block.reading):
            self.temp_plot_data = [temp_data_buffer[self.temp_start:-1], temp_time_buffer[self.temp_start:-1]]
            self.temp_plot_avg = sum(self.temp_plot_data[0]) / len(self.temp_plot_data[0])
            self.temp_plot_data[0].insert(0, self.temp_plot_avg)
            self.temp_plot_data[1].insert(0, -1)
            self.temp_time = 1
            temp_data_buffer.clear()
            temp_time_buffer.clear()

        # PMT block - updated to match ph1 pattern
        if self.left_middle_item2.display or self.left_middle_item2.site1_block.reading:
            if self.pmt_time == 1:
                pmt_time_buffer = []
                pmt_data_buffer = []
            pmt_time_buffer.append(self.pmt_time)
            pmt_graph_data = float(msg[4][1])
            pmt_data_buffer.append(pmt_graph_data)
            self.pmt_time += 1
        if self.left_middle_item2.site1_block.start_read:
            self.pmt_start = len(pmt_data_buffer)
            self.left_middle_item2.site1_block.start_read = False
        if not (self.left_middle_item2.site1_block.start_read or self.left_middle_item2.site1_block.reading):
            self.pmt_plot_data = [pmt_data_buffer[self.pmt_start:-1], pmt_time_buffer[self.pmt_start:-1]]
            self.pmt_plot_avg = sum(self.pmt_plot_data[0]) / len(self.pmt_plot_data[0])
            self.pmt_plot_data[0].insert(0, self.pmt_plot_avg)
            self.pmt_plot_data[1].insert(0, -1)
            self.pmt_time = 1
            pmt_data_buffer.clear()
            pmt_time_buffer.clear()
        
        self.pmt_switch = msg[5][1]
        self.left_middle_item2.site1_block.pmtWidget.setText(f"PMT: {self.pmt_switch}")

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
        self.display = False

    def read(self):
        print("Read Sensor")
        self.reading = not self.reading
        self.start_read = self.reading
        if self.reading:
            self.read_button.setStyleSheet("background-color: #FF0000;")
        else:
            self.read_button.setStyleSheet("background-color: #FFFFFF;")
    
    def save(self):
        """Save temperature and humidity data to CSV and graphs to PNG"""
        print("Saving sensor data...")
        
        # Create directory if it doesn't exist
        csv_dir = os.path.join(os.path.expanduser("~"), "rover-ws/src/rsx-rover/science_data")
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        
        # Create a timestamp for unique filenames
        timestamp = rospy.Time.now().to_sec()
        timestr = time.strftime("%Y%m%d-%H%M%S")
        
        # Save temperature data to CSV if available
        if self.ui.temp_plot_data:
            # Save temperature data to CSV
            temp_csv_path = os.path.join(csv_dir, f"temperature_data_{timestr}.csv")
            with open(temp_csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Temperature (°C)"])
                for i in range(len(self.ui.temp_plot_data[0])):
                    writer.writerow([self.ui.temp_plot_data[1][i], self.ui.temp_plot_data[0][i]])
            
            # Save temperature graph as PNG
            temp_png_path = os.path.join(csv_dir, f"temperature_graph_{timestr}.png")
            exporter = pg.exporters.ImageExporter(self.ui.science_temperature_plot.plotItem)
            exporter.export(temp_png_path)
            
            print(f"Temperature data saved to {temp_csv_path}")
            print(f"Temperature graph saved to {temp_png_path}")
        
        # Save humidity data to CSV if available
        if self.ui.hum_plot_data:
            # Save humidity data to CSV
            hum_csv_path = os.path.join(csv_dir, f"humidity_data_{timestr}.csv")
            with open(hum_csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Humidity (%)"])
                for i in range(len(self.ui.hum_plot_data[0])):
                    writer.writerow([self.ui.hum_plot_data[1][i], self.ui.hum_plot_data[0][i]])
            
            # Save humidity graph as PNG
            hum_png_path = os.path.join(csv_dir, f"humidity_graph_{timestr}.png")
            exporter = pg.exporters.ImageExporter(self.ui.science_humidity_plot.plotItem)
            exporter.export(hum_png_path)
            
            print(f"Humidity data saved to {hum_csv_path}")
            print(f"Humidity graph saved to {hum_png_path}")
        
        # Show a confirmation message
        if self.ui.temp_plot_data or self.ui.hum_plot_data:
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

    def sample(self):
        if self.chem:
            if self.site1:
                self.ui.science_serial_controller.publish("<A>")
                print("Sampled Chem Site 1")
            else:
                self.ui.science_serial_controller.publish("<B>")
                print("Sampled Chem Site 2")
        else:
            if self.site1:
                self.ui.science_serial_controller.publish("<D>")
                print("Sampled FAM Site 1")
            else:
                self.ui.science_serial_controller.publish("<E>")
                print("Sampled FAM Site 2")
    
    def read(self):
        self.reading = not self.reading
        self.start_read = self.reading
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
        csv_dir = os.path.join(os.path.expanduser("~"), "rover-ws/src/rsx-rover/science_data")
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        
        # Create a timestamp for unique filenames
        timestamp = rospy.Time.now().to_sec()
        timestr = time.strftime("%Y%m%d-%H%M%S")
        
        # Determine which data to save based on chemistry/FAM and site
        if self.chem:
            if self.site1:
                data = self.ui.ph1_plot_data
                plot_widget = self.ui.science_ph1_plot
                data_type = "ph1"
                title = "Site 1 pH"
            else:
                data = self.ui.ph2_plot_data
                plot_widget = self.ui.science_ph2_plot
                data_type = "ph2"
                title = "Site 2 pH"
        else:
            # FAM/Soil data (PMT)
            data = self.ui.pmt_plot_data
            plot_widget = self.ui.science_pmt_plot
            data_type = "pmt"
            title = "Site " + self.ui.pmt_switch + " PMT"
        
        # Save data to CSV if available
        if data:
            # Save data to CSV
            csv_path = os.path.join(csv_dir, f"{data_type}_data_{timestr}.csv")
            with open(csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", f"{title} Value"])
                for i in range(len(data[0])):
                    writer.writerow([data[1][i], data[0][i]])
            
            # Save graph as PNG
            png_path = os.path.join(csv_dir, f"{data_type}_graph_{timestr}.png")
            exporter = pg.exporters.ImageExporter(plot_widget.plotItem)
            exporter.export(png_path)
            
            print(f"{title} data saved to {csv_path}")
            print(f"{title} graph saved to {png_path}")
            
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
    def __init__(self, ui: RoverGUI):
        super().__init__()
        
        # Create a proper layout
        self.layout = QVBoxLayout()
        
        self.ui = ui

        # Create a row layout for buttons
        button_layout = QHBoxLayout()

        microscope_layout = QVBoxLayout()
        
        # Create buttons
        self.take_pano_button = QPushButton("Take Pano")
        self.toggle_genie_button = QPushButton("Toggle Genie Filter")
        self.microscope_zoom_in_button = QPushButton("+")
        self.microscope_zoom_out_button = QPushButton("-")
        self.micro_slider_splitter = Slider(Qt.Horizontal)
        
        # Set button styles
        button_style = "padding: 4px; min-height: 20px;"
        self.take_pano_button.setStyleSheet(button_style)
        self.toggle_genie_button.setStyleSheet(button_style)
        self.microscope_zoom_in_button.setStyleSheet(button_style)
        self.microscope_zoom_out_button.setStyleSheet(button_style)
        self.micro_slider_splitter.setMinimum(0)
        self.micro_slider_splitter.setMaximum(100)
        self.micro_slider_splitter.setValue(50)

        # Set size policies to make buttons fit their content
        self.take_pano_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.toggle_genie_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
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
        
        # Add stretch FIRST to push buttons to the right
        button_layout.addStretch(1)

        # Add buttons to the row layout (now they'll be right-aligned)
        button_layout.addWidget(self.take_pano_button)
        button_layout.addWidget(self.toggle_genie_button)
        button_layout.addWidget(self.micro_slider_splitter)
        button_layout.addWidget(microscope_widget)
        
        # Add the button row to the main layout
        self.layout.addLayout(button_layout)
        
        # Apply the layout to the widget
        self.setLayout(self.layout)

        self.zoom_mag = 5

        self.pano_control = rospy.Publisher('/pano_control', Bool, queue_size=10)
        self.pano_result = rospy.Subscriber('/pano_result', Image, self.pano_callback)

        self.show_pano_button(False)
        # self.show_genie_button(False)
        self.show_zoom_controls(False)

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
    
    def take_pano(self):
        # Implement the logic to take a panorama
        print("Taking panorama...")
        self.pano_control.publish(True)
    
    def toggle_genie(self):
        # Implement the logic to toggle the genie lens
        self.ui.science_serial_controller.publish("<M>")
        print("Toggling genie filter...")
    
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
            pano_dir = os.path.join(os.path.expanduser("~"), "rover-ws/src/rsx-rover/science_data", "panoramas")
            if not os.path.exists(pano_dir):
                os.makedirs(pano_dir)
            
            # Create a timestamp for a unique filename
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(pano_dir, f"panorama_{timestr}.png")
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Notify user
            print(f"Panorama saved to {filename}")
            
            # Flash the button to indicate successful save
            original_style = self.take_pano_button.styleSheet()
            self.take_pano_button.setStyleSheet("background-color: #00FF00; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.take_pano_button.setStyleSheet(original_style))
            
        except Exception as e:
            print(f"Error saving panorama: {e}")
            # Flash the button red to indicate failure
            original_style = self.take_pano_button.styleSheet()
            self.take_pano_button.setStyleSheet("background-color: #FF0000; padding: 4px; min-height: 20px;")
            QTimer.singleShot(1000, lambda: self.take_pano_button.setStyleSheet(original_style))


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