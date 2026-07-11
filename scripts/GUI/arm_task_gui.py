#!/usr/bin/env python3


import sys
import os

# Import cv2 first, then fix the Qt plugin path it sets
import cv2
# Remove the Qt plugin path that cv2 sets to avoid conflicts with PyQt5
if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]
import rclpy
from rclpy.node import Node
import map_viewer as map_viewer
from map_viewer import MapPoint
from pathlib import Path
import numpy as np


from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, \
    QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame, \
    QCheckBox,QSplitter,QStylePainter, QStyleOptionComboBox, QStyle, \
    QToolButton, QMenu, QLineEdit , QPushButton, QTextEdit,\
    QListWidget, QListWidgetItem, QStyleOptionSlider, QSizePolicy
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String, Bool
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap, QPainter,QPalette,QStandardItemModel, QTextCursor, QFont
from calian_gnss_ros2_msg.msg import GnssSignalStatus

#cache folder of map tiles generated from tile_scraper.py
# CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"
CACHE_DIR = Path(__file__).parent.parent.parent.parent.parent.resolve() / "src/rsx-rover/scripts/GUI/tile_cache"

DRAW_RADIUS_METERS = 20.0


def _parse_bool(value) -> bool:
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "t", "yes", "y"}

#map widget that has map viewer 
class mapOverlay(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.viewer = map_viewer.MapViewer()
        #sets the source of map tiles to local tile cache folder
        self.viewer.set_map_server(
            str(CACHE_DIR) + '/arcgis_world_imagery/{z}/{y}/{x}.jpg', 19
        )
        self.setLayout(self.initOverlayout())
        self.centreOnRover = False
        
        # ROS Subscriber for GPS coordinates
        
        self.node.create_subscription(NavSatFix, '/calian_gnss/gps', self.update_gps_coordinates, 10)
        self.node.create_subscription(GnssSignalStatus, '/calian_gnss/gps_extended', self.update_gps_heading, 10)
    
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

    def __init__(self, node):
        super().__init__()
        self.init_ui()
        self.node = node
        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        # rospy.Subscriber('aruco_found', Bool, self.bool_callback)
        # rospy.Subscriber('aruco_name', String, self.string_callback)
        self.node.create_subscription(Bool, 'aruco_found', self.bool_callback, 10)
        self.node.create_subscription(String, 'aruco_name', self.string_callback, 10)

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
        self.node = node
        # Connect signals to the corresponding update methods
        self.update_label_signal.connect(self.update_label)
        self.update_list_signal.connect(self.update_string_list)

        # Initialize ROS subscribers
        # rospy.Subscriber('aruco_found', Bool, self.bool_callback)
        # rospy.Subscriber('aruco_name', String, self.string_callback)
        self.node.create_subscription(Bool, 'aruco_found', self.bool_callback, 10)
        self.node.create_subscription(String, 'aruco_name', self.string_callback, 10)
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
        self.node=node
        # Connect signals to the corresponding update methods
        self.update_mallet_signal.connect(self.update_mallet)
        self.update_bottle_signal.connect(self.update_bottle)

        # Initialize ROS subscribers
        self.node.create_subscription(Bool, 'mallet_detected', self.mallet_callback, 10)
        self.node.create_subscription(Bool, 'waterbottle_detected', self.bottle_callback, 10)
        
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
        self.node = node
        # Connect the signal to the update method
        self.update_label_signal.connect(self.update_label)

        # Initialize ROS subscriber
        self.node.create_subscription(String, '/led_colour', self.callback, 10)

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
    def __init__(self, map_overlay, node):
        super().__init__()
        self.node = node
        # self.longLat_pub = rospy.Publisher('/long_lat_goal_array', Float32MultiArray, queue_size=5)
        self.longLat_pub = self.node.create_publisher(Float32MultiArray, '/long_lat_goal_array', 5)
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
    def __init__(self, map_overlay, node):
        super().__init__()
        self.node = node
 
        self.longLat_pub = self.node.create_publisher(Float32MultiArray, '/long_lat_goal_array', 5)
        self.array = Float32MultiArray()
        self.radius_flags = []
        layout = QVBoxLayout(self)

        self.submit_button = QPushButton("Get Data From File")
        self.submit_button.clicked.connect(self.collect_data)
        self.submit_button.clicked.connect(self.plot_points)
        layout.addWidget(self.submit_button)
        self.setLayout(layout)

        self.viewer = map_overlay

        self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')
        self.radius_layer_name = "goal_radius"
        if self.radius_layer_name not in self.viewer.viewer.points_layer:
            self.viewer.viewer.add_point_layer(self.radius_layer_name, 'red', 'transparent', 'transparent')

    def collect_data(self):
        # Read data from the file
        file_path = Path(__file__).parent.parent.parent.resolve() / "long_lat_goal.csv"
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Process each line and publish
        data = []
        self.radius_flags = []
        for line in lines:
            parts = [part.strip() for part in line.strip().split(',')]
            if len(parts) < 3:
                continue
            try:
                lat = float(parts[1])
                lng = float(parts[2])
            except ValueError:
                continue
            data.extend([lat, lng])
            draw_radius = _parse_bool(parts[3]) if len(parts) > 3 else False
            self.radius_flags.append(draw_radius)
        self.array.data = data
        self.longLat_pub.publish(self.array)
        print("Published Data:", data)

    def plot_points(self):
        data = self.array.data
        print("Plotting points from file data")
        radius_points = []
        
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
                draw_radius = False
                if i // 2 < len(self.radius_flags):
                    draw_radius = self.radius_flags[i // 2]
                if draw_radius:
                    radius_points.append(map_viewer.MapPoint(lat, lng, DRAW_RADIUS_METERS, point_name))

        self.viewer.viewer.set_points(self.radius_layer_name, radius_points)


class LngLatDeliveryEntryFromFile(QWidget):
    def __init__(self, map_overlay):
        super().__init__()
        self.array = Float32MultiArray()
        self.radius_flags = []
        layout = QVBoxLayout(self)

        self.submit_button = QPushButton("Get Delivery Data From File")
        self.submit_button.clicked.connect(self.collect_data)
        layout.addWidget(self.submit_button)
        self.setLayout(layout)

        self.viewer = map_overlay

        self.viewer.viewer.add_point_layer('gps_points', 'green', 'green', 'yellow')
        self.radius_layer_name = "delivery_radius"
        if self.radius_layer_name not in self.viewer.viewer.points_layer:
            self.viewer.viewer.add_point_layer(self.radius_layer_name, 'red', 'transparent', 'transparent')

    def collect_data(self):
        # Read data from the file
        file_path = Path("~/rover_ws/src/rsx-rover/delivery_lat_lon_goal.csv").expanduser()
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Process each line and publish
        data = []
        self.radius_flags = []
        for line in lines:
            parts = [part.strip() for part in line.strip().split(',')]
            if len(parts) < 3:
                continue
            name = parts[0]
            try:
                lat = float(parts[1])
                lng = float(parts[2])
            except ValueError:
                continue
            data.append(name)
            data.append(lat)
            data.append(lng)
            draw_radius = _parse_bool(parts[3]) if len(parts) > 3 else False
            self.radius_flags.append(draw_radius)
        print("Plotting points from file data")
        print(data)
        
        # Create MapPoint objects from the data (pairs of lat, lng values)
        # map_points = []
        radius_points = []
        for i in range(0, len(data), 3):
            if i + 1 < len(data):  # Ensure we have both lat and lng and they're not zero
                lat = data[i+1]
                lng = data[i+2]
                # Create a MapPoint with a radius of 5 and a name based on index
                point_name = data[i]
                # map_points.append((lat, lng, 5, point_name))
                print(f"{point_name}: {lat}, {lng}")
                self.viewer.viewer.add_goal(point_name, lat, lng)
                draw_radius = False
                if i // 3 < len(self.radius_flags):
                    draw_radius = self.radius_flags[i // 3]
                if draw_radius:
                    radius_points.append(map_viewer.MapPoint(lat, lng, DRAW_RADIUS_METERS, str(point_name)))

        self.viewer.viewer.set_points(self.radius_layer_name, radius_points)
           

class VelocityControl:
    def __init__(self, node):
        self.node=node
        # self.pub = rospy.Publisher('/drive', Twist, queue_size=10)
        self.pub = self.node.create_publisher(Twist, '/drive', 10)
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
        self.setAlignment(Qt.AlignCenter)  # Center content

    def resizeEvent(self, event):
        if self.pixmap() and not self.pixmap().isNull():
            self.setPixmap(self.pixmap().scaled(
                self.width(), 
                self.height(),
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            ))
        super().resizeEvent(event)


class CameraFeed:
    def __init__(self, node, label1, label2, label3, label5, splitter):
        self.node = node
        self.last_genie_image = None
        self.active_cameras = {"Zed (front) camera": False, "Butt camera": False, "Microscope camera": False, "Genie camera": False, "Webcam": False}
        self.bridge = CvBridge()
        self.image_sub1 = None
        self.image_sub2 = None
        self.image_sub3 = None
        self.image_sub4 = None
        self.image_sub5 = None
        self.state_sub = self.node.create_subscription(String, "state", self.state_callback, 10)
        self.bbox_sub = self.node.create_subscription(Float64MultiArray, "aruco_node/bbox", self.bbox_callback, 10) 
        self.obj_bbox = self.node.create_subscription(Float64MultiArray, "object/bbox", self.bbox_callback, 10)
        
        self.label1 = label1
        self.label2 = label2
        self.label3 = label3
        self.label5 = label5
        self.splitter = splitter

        # Set size policies correctly
        self.label1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label5.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label1.setMinimumSize(300, 200)
        self.label2.setMinimumSize(300, 200)
        self.label3.setMinimumSize(300, 200)
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
        self.label5.hide()

    def register_subscriber1(self):
        if self.image_sub1 is None:
            # self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color/compressed", CompressedImage, self.callback1)
            self.image_sub1 = self.node.create_subscription(CompressedImage, "/zed/zed_node/rgb/image_rect_color/compressed", self.callback1, 10)
    def unregister_subscriber1(self):
        if self.image_sub1:
            self.image_sub1.unregister()
            self.image_sub1 = None

    def register_subscriber2(self):
        if self.image_sub2 is None:
            # self.image_sub2 = rospy.Subscriber("/camera2/camera/color/image_raw/compressed", CompressedImage, self.callback2)
            self.image_sub2 = self.node.create_subscription(CompressedImage, "/camera1/color/image_raw/compressed", self.callback2, 10)
    def unregister_subscriber2(self):
        if self.image_sub2:
            self.image_sub2.unregister()
            self.image_sub2 = None

    def register_subscriber3(self):
        if self.image_sub3 is None:
            # self.image_sub3 = rospy.Subscriber("/webcam/compressed", CompressedImage, self.callback3)
            self.image_sub3 = self.node.create_subscription(CompressedImage, "/webcam/compressed", self.callback3, 10)
    def unregister_subscriber3(self):
        if self.image_sub3:
            self.image_sub3.unregister()
            self.image_sub3 = None

    def register_subscriber5(self):
        if self.image_sub5 is None:
            # self.image_sub5 = rospy.Subscriber("/webcam2/compressed", CompressedImage, self.callback5)
            self.image_sub5 = self.node.create_subscription(CompressedImage, "/webcam2/compressed", self.callback5, 10)
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
        if self.active_cameras["Webcam Left"]:
            # print("get image")
            self.update_image(data, self.label3)

    def callback5(self, data):
        if self.active_cameras["Webcam Right"]:
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

        if self.active_cameras["Webcam Left"]:
            self.register_subscriber3()
        else:
            self.unregister_subscriber3()

        if self.active_cameras["Webcam Right"]:
            self.register_subscriber5()
        else:
            self.unregister_subscriber5()

    def update_visibility(self):
        active_indexes = []
        if self.active_cameras["Zed (front) camera"]:
            self.label1.show()
            self.splitter.setStretchFactor(0, 1)
            active_indexes.append(0)
        else:
            self.label1.hide()
            self.splitter.setStretchFactor(0, 0)
        if self.active_cameras["Butt camera"]:
            self.label2.show()
            self.splitter.setStretchFactor(1, 1)
            active_indexes.append(1)
        else:
            self.label2.hide()
            self.splitter.setStretchFactor(1, 0)
        if self.active_cameras["Webcam Left"]:
            self.label3.show()
            self.splitter.setStretchFactor(2, 1)
            active_indexes.append(2)
        else:
            self.label3.hide()
            self.splitter.setStretchFactor(2, 0)
        if self.active_cameras["Webcam Right"]:
            self.label5.show()
            self.splitter.setStretchFactor(3, 1)
            active_indexes.append(3)
        else:
            self.label5.hide()
            self.splitter.setStretchFactor(3, 0)

        if active_indexes:
            self._apply_splitter_sizes(active_indexes)

    def _apply_splitter_sizes(self, active_indexes):
        orientation = self.splitter.orientation()
        total = self.splitter.size().width() if orientation == Qt.Horizontal else self.splitter.size().height()
        if total <= 0:
            total = max(len(active_indexes), 1) * 100

        per_size = max(int(total / len(active_indexes)), 1)
        sizes = [0, 0, 0, 0]
        for idx in active_indexes:
            sizes[idx] = per_size

        # Force the active feeds to take all available space in the splitter.
        self.splitter.setSizes(sizes)


#main gui class, make updates here to change top level hierarchy
class RoverGUI(QMainWindow):
    statusSignal = pyqtSignal(str)
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.velocity_control = VelocityControl(self.node)
        # self.gui_status_sub = rospy.Subscriber('gui_status', String, self.string_callback)
        # self.auto_abort_pub = rospy.Publisher('/auto_abort_check', Bool, queue_size=5)
        self.gui_status_sub = self.node.create_subscription(String, 'gui_status', self.string_callback, 10)
        self.auto_abort_pub = self.node.create_publisher(Bool, '/auto_abort_check', 5)
        # self.manual_abort_pub = rospy.Publisher('/manual_abort_check', Bool, queue_size=5)
        # self.next_state_pub = rospy.Publisher('/next_state', Bool, queue_size=5)
        self.next_state_pub = self.node.create_publisher(Bool, '/next_state', 5)
        self.reached_state = None

        # Create tab
        self.split_screen_tab = QWidget()
        self.longlat_tab = QWidget()
        self.camsTab = QWidget()

        # Add tab to QTabWidget
        self.tabs.addTab(self.split_screen_tab, "Main Gui")
        self.tabs.addTab(self.longlat_tab, "State Machine")
        self.tabs.addTab(self.camsTab, "Cameras")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)

        
        self.statusSignal.connect(self.string_signal_receive)
        self.setup_split_screen_tab()
        self.setup_lngLat_tab()
        self.setup_cams_tab()
        

    def string_callback(self, msg):
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

        # ROS functionality
        self.camerasplitter_cams_tab = QSplitter(Qt.Horizontal)
        self.camera_feed_cams_tab = CameraFeed(self.node, self.camera_label1_cams_tab, self.camera_label2_cams_tab, self.camera_label3_cams_tab, self.camera_label4_cams_tab, self.camerasplitter_cams_tab)
        self.camerasplitter_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        # Use CameraSelect menu-based selector
        self.camera_selector_cams_tab = CameraSelect()
        self.camera_selector_cams_tab.cameras_changed.connect(self.camera_feed_cams_tab.update_active_cameras)

        camera_layout.addWidget(self.camera_selector_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label1_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label2_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label3_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label4_cams_tab)
        camera_layout.addWidget(self.camerasplitter_cams_tab)
        camera_layout.setStretch(0, 0)
        camera_layout.setStretch(1, 1)

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
        self.lngLatEntry = LngLatEntryBar(self.map_overlay_splitter, self.node)
        self.lngLatFile = LngLatEntryFromFile(self.map_overlay_splitter, self.node)
        self.lngLatDeliveryFile = LngLatDeliveryEntryFromFile(self.map_overlay_splitter)
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
        if hasattr(self, "joystick_splitter"):
            self.joystick_splitter.setJoystickPosition(x, y)
        if hasattr(self, "joystick_control"):
            self.joystick_control.setJoystickPosition(x, y)
        # add more joystick instances here to sync

    #used to initialize main tab with splitters
    def setup_split_screen_tab(self):
        button_style = "padding: 4px; min-height: 20px;"
        splitter = QSplitter(Qt.Horizontal)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

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

        # ROS functionality
        self.camerasplitter = QSplitter(Qt.Vertical)
        self.camera_feed = CameraFeed(self.node, self.camera_label1, self.camera_label2, self.camera_label3, self.camera_label4, self.camerasplitter)
        self.camerasplitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        # Use CameraSelect menu-based selector
        self.camera_selector = CameraSelect()
        self.camera_selector.cameras_changed.connect(self.camera_feed.update_active_cameras)

        camera_layout.addWidget(self.camera_selector)
        self.camerasplitter.addWidget(self.camera_label1)
        self.camerasplitter.addWidget(self.camera_label2)
        self.camerasplitter.addWidget(self.camera_label3) 
        self.camerasplitter.addWidget(self.camera_label4)
        self.camera_label3.hide()  # Hide these initially
        self.camera_label4.hide()
        camera_layout.addWidget(self.camerasplitter)
        camera_layout.setStretch(0, 0)
        camera_layout.setStretch(1, 1)

        # camera_layout.addWidget(self.camera_label_splitter)
        camera_group.setLayout(camera_layout)

        # Add map to the splitter
        map_group = QGroupBox("Map")
        map_layout = QVBoxLayout()
        self.map_overlay_splitter = mapOverlay(self.node)
        self.map_overlay_splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.checkbox_setting_splitter = QCheckBox("Recentre map when rover offscreen")
        self.checkbox_setting_splitter.setChecked(False)  # Set the default state to unchecked
        # self.checkbox_setting_splitter.stateChanged.connect(self.on_checkbox_state_changed)
        self.checkbox_setting_splitter.stateChanged.connect(
            lambda state: self.on_checkbox_state_changed(state, self.map_overlay_splitter)
        )
        self.clear_map_button = QPushButton("Clear Map")
        self.clear_map_button.setStyleSheet(button_style)
        self.clear_map_button.clicked.connect(self.map_overlay_splitter.clear_map)

        # Create horizontal layout for checkbox and clear button
        checkbox_layout = QHBoxLayout()
        checkbox_layout.addWidget(self.checkbox_setting_splitter)
        checkbox_layout.addStretch(1)  # This pushes the checkbox left and button right
        checkbox_layout.addWidget(self.clear_map_button)
        
        # Create a container widget for the checkbox layout
        checkbox_container = QWidget()
        checkbox_container.setLayout(checkbox_layout)
        
        # Add the container to the map layout instead of individual widgets
        map_layout.addWidget(checkbox_container)
        map_layout.addWidget(self.map_overlay_splitter)
        map_group.setLayout(map_layout)

        splitter.addWidget(camera_group)
        splitter.addWidget(map_group)
        
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([600, 600])
        
        split_screen_layout = QVBoxLayout()
        split_screen_layout.addWidget(splitter)
        self.split_screen_tab.setLayout(split_screen_layout)

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
        if hasattr(self, "gear_slider_splitter"):
            self.gear_slider_splitter.setValue(value)
    
    def change_exposure(self,value):
        #self.camera_feed_cams_tab.exposure_value = value
        self.camera_feed.exposure_value = value

    def pub_next_state(self):
        self.next_state_pub.publish(True)

    # def pub_manual_abort(self):
    #     self.manual_abort_pub.publish(True)

    def pub_auto_abort(self):
        self.auto_abort_pub.publish(True)


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
        self.layout = QVBoxLayout()

        self.toolButton = QToolButton(self)
        self.toolButton.setText("Cameras List")
        self.toolMenu = QMenu(self)

        # Remove microscope and genie camera from the list
        self.cameras = ["Zed (front) camera", "Butt camera", "Webcam Left", "Webcam Right"]
        self.selected_cameras = {camera: False for camera in self.cameras}

        for camera in self.cameras:
            action = self.toolMenu.addAction(camera)
            action.setCheckable(True)
            self.toolMenu.triggered.connect(self.handle_camera_selection)

        self.toolButton.setMenu(self.toolMenu)
        self.toolButton.setPopupMode(QToolButton.InstantPopup)
        self.layout.addWidget(self.toolButton)
        self.layout.addStretch()
        self.setLayout(self.layout)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

    def handle_camera_selection(self, action):
        camera = action.text()
        self.selected_cameras[camera] = action.isChecked()
        self.cameras_changed.emit(self.selected_cameras)


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('rover_gui')
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
    # Create a QTimer to spin ROS2 in the Qt event loop
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)  # Spin every 10ms

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()