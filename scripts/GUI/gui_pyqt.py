#!/usr/bin/env python3


import sys
import rospy
import map_viewer as map_viewer
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
from sensor_msgs.msg import NavSatFix, CompressedImage
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String, Bool
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter,QPalette,QStandardItemModel, QTextCursor, QFont

#cache folder of map tiles generated from tile_scraper.py
CACHE_DIR = Path(__file__).parent.resolve() / "tile_cache"

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
        self.label.setFont(QFont("Arial", 16, QFont.Bold))

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
    def __init__(self, label1, label2, splitter):
        self.bridge = CvBridge()
        self.image_sub1 = None
        self.image_sub2 = None
        self.state_sub = rospy.Subscriber("state", String, self.state_callback)

        self.obj_bbox = rospy.Subscriber("object/bbox", Float64MultiArray, self.bbox_callback)
        self.bbox_sub = rospy.Subscriber("aruco_node/bbox", Float64MultiArray, self.bbox_callback)

        self.label1 = label1
        self.label2 = label2
        self.splitter = splitter

        # Set size policies correctly
        self.label1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label1.setMinimumSize(300, 200)
        self.label2.setMinimumSize(300, 200)
        self.splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.active_cameras = {"Zed (front) camera": False, "Butt camera": False}
        self.bbox = None  

        self.bbox_timer = QTimer()
        self.bbox_timer.setInterval(1000)
        self.bbox_timer.timeout.connect(self.clear_bbox)
        self.bbox_timer.setSingleShot(True)

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

    def update_image(self, data, label):
        """Decode and update the camera image with bounding box."""
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            return  

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

    def update_visibility(self):
        active_count = sum(self.active_cameras.values())
        if active_count == 0:
            self.label1.hide()
            self.label2.hide()
        elif active_count == 1:
            if self.active_cameras["Zed (front) camera"]:
                self.label1.show()
                self.label2.hide()
                self.splitter.setStretchFactor(0, 1)
                self.splitter.setStretchFactor(1, 0)
            else:
                self.label1.hide()
                self.label2.show()
                self.splitter.setStretchFactor(0, 0)
                self.splitter.setStretchFactor(1, 1)
        else:  
            self.label1.show()
            self.label2.show()
            self.splitter.setStretchFactor(0, 1)
            self.splitter.setStretchFactor(1, 1)

#main gui class, make updates here to change top level hierarchy
class RoverGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        # Initialize QTabWidget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.velocity_control = VelocityControl()
        self.gui_status_sub = rospy.Subscriber('gui_status', String, self.string_callback)
        self.auto_abort_pub = rospy.Publisher('/auto_abort_check', Bool, queue_size=5)
        # self.manual_abort_pub = rospy.Publisher('/manual_abort_check', Bool, queue_size=5)
        self.next_state_pub = rospy.Publisher('/next_state', Bool, queue_size=5)
        self.reached_state = None

        # Create tab
        self.split_screen_tab = QWidget()
        self.longlat_tab = QWidget()
        self.controlTab = QWidget()
        self.camsTab = QWidget()

        # Add tab to QTabWidget
        self.tabs.addTab(self.split_screen_tab, "Main Gui")
        self.tabs.addTab(self.longlat_tab, "State Machine")
        self.tabs.addTab(self.controlTab, "Controls")
        self.tabs.addTab(self.camsTab, "Cameras")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)


        self.setup_control_tab()
        self.setup_split_screen_tab()
        self.setup_lngLat_tab()
        self.setup_cams_tab()
        

    def string_callback(self, msg):
        self.statusTerminal.string_callback(msg)
        msg_list = msg.data.split(" ")
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
        else:
            self.setStyleSheet("background-color: #FFFFFF")
            self.status_label.setText("")


        
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
        # ROS functionality
        self.camerasplitter_cams_tab = QSplitter(Qt.Horizontal)
        self.camera_feed_cams_tab = CameraFeed(self.camera_label1_cams_tab, self.camera_label2_cams_tab,self.camerasplitter_cams_tab)
        self.camerasplitter_cams_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        # Use CameraSelect menu-based selector
        self.camera_selector_cams_tab = CameraSelect()
        self.camera_selector_cams_tab.cameras_changed.connect(self.camera_feed_cams_tab.update_active_cameras)

        camera_layout.addWidget(self.camera_selector_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label1_cams_tab)
        self.camerasplitter_cams_tab.addWidget(self.camera_label2_cams_tab)
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

        # Sync joystick movements between tabs
        self.joystick_control.joystickMoved.connect(self.sync_joysticks)

        # Add to layout
        controls_layout.addWidget(joystick_group)
        controls_layout.addWidget(gear_group)
        self.controls_group.setLayout(controls_layout)

        control_tab_layout = QVBoxLayout()
        control_tab_layout.addWidget(self.controls_group)
        self.controlTab.setLayout(control_tab_layout)

        
        # self.controlTab.setLayout(control_tab_layout)

    def setup_lngLat_tab(self):
        self.lngLatEntry = LngLatEntryBar(self.map_overlay_splitter)
        self.lngLatFile = LngLatEntryFromFile(self.map_overlay_splitter)
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
        Lnglat_tab_layout.addWidget(self.stateMachineDialog)
        Lnglat_tab_layout.addWidget(self.arucoGroupBox) 
        

        self.longlat_tab.setLayout(Lnglat_tab_layout)
    def sync_joysticks(self, x, y):
        """Sync joystick movement between tabs"""
        self.joystick_splitter.setJoystickPosition(x, y)
        self.joystick_control.setJoystickPosition(x, y)
        # add more joystick instances here to sync

    #used to initialize main tab with splitters
    def setup_split_screen_tab(self):
        # Controls section
        self.controls_group = QGroupBox("Controls")
        controls_layout = QHBoxLayout()

        # Joystick
        self.joystick_splitter = Joystick(self.velocity_control)
        joystick_group = QGroupBox("Joystick")
        joystick_layout = QVBoxLayout()
        joystick_layout.addWidget(self.joystick_splitter)
        joystick_group.setLayout(joystick_layout)
        self.joystick_splitter.joystickMoved.connect(self.sync_joysticks)


        # Gear slider
        gear_group = QGroupBox("Gear Control")
        slider_layout = QVBoxLayout()
        # make a new slider object 
        self.gear_slider_splitter = Slider(Qt.Horizontal)
        self.gear_slider_splitter.setMinimum(0)
        self.gear_slider_splitter.setMaximum(100)
        self.gear_slider_splitter.setValue(0)


        # Connect the signal to the function
        self.gear_slider_splitter.valueUpdated.connect(self.change_gear)
        
        slider_layout.addWidget(self.gear_slider_splitter)
        gear_group.setLayout(slider_layout)

        # Add joystick and gear controls side by side
        controls_layout.addWidget(joystick_group)
        controls_layout.addWidget(gear_group)
        self.controls_group.setMinimumHeight(100)

        self.controls_group.setLayout(controls_layout)
        # vertical_splitter.addWidget(controls_group)
        control_tab_layout = QVBoxLayout()
        control_tab_layout.addWidget(self.controls_group)

        # Create the new section to appear above camera
        status_group = QGroupBox("Status")
        status_layout = QHBoxLayout()  # Changed from QVBoxLayout to QHBoxLayout
        status_group.setMaximumHeight(100)  # Set maximum height
        
        # Add widgets to the new section
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("font-size: 64px")  
        # self.next_state_input = QLineEdit()
        # self.next_state_input.setPlaceholderText("Enter next state")
        # self.next_state_input.setStyleSheet("font-size: 16px")
        # self.next_state_input.setFixedWidth(150)  # Adjust width as needed
        next_button = QPushButton("Next Task")
        # manual_abort_button = QPushButton("Manual Abort")
        auto_abort_button = QPushButton("Auto Abort")
        
        # Make buttons smaller to match the reduced section height
        button_style = "padding: 4px; min-height: 20px;"
        next_button.setStyleSheet(button_style)
        next_button.clicked.connect(self.pub_next_state)
        # manual_abort_button.setStyleSheet(button_style)
        # manual_abort_button.clicked.connect(self.pub_manual_abort)
        auto_abort_button.setStyleSheet(button_style)
        auto_abort_button.clicked.connect(self.pub_auto_abort)
        
        # Add widgets to layout horizontally
        status_layout.addWidget(self.status_label)
        # status_layout.addWidget(self.next_state_input)  # Add input field before button
        status_layout.addWidget(next_button)
        # status_layout.addWidget(manual_abort_button)
        status_layout.addWidget(auto_abort_button)
        status_group.setLayout(status_layout)

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
        # ROS functionality
        self.camerasplitter = QSplitter(Qt.Horizontal)
        self.camera_feed = CameraFeed(self.camera_label1, self.camera_label2,self.camerasplitter)
        self.camerasplitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        

        # Use CameraSelect menu-based selector
        self.camera_selector = CameraSelect()
        self.camera_selector.cameras_changed.connect(self.camera_feed.update_active_cameras)

        camera_layout.addWidget(self.camera_selector)
        self.camerasplitter.addWidget(self.camera_label1)
        self.camerasplitter.addWidget(self.camera_label2)
        camera_layout.addWidget(self.camerasplitter)

        # camera_layout.addWidget(self.camera_label_splitter)
        camera_group.setLayout(camera_layout)

        # Add map to the splitter
        map_group = QGroupBox("Map")
        map_layout = QVBoxLayout()
        self.map_overlay_splitter = mapOverlay()
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

        # Create a vertical splitter for the left side
        left_side_splitter = QSplitter(Qt.Vertical)
        left_side_splitter.addWidget(status_group)
        left_side_splitter.addWidget(camera_group)

        # Create the horizontal splitter for the main layout
        splitter.addWidget(left_side_splitter)  # Left side has new section stacked above camera
        splitter.addWidget(map_group)           # Right side has map
        
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        
        # Create a group box for the status terminal
        vertSplitter = QSplitter(Qt.Vertical)
        vertSplitter.addWidget(splitter)
        self.statusTerminal = statusTerminal()
        self.statusTermGroupBox = QGroupBox("Status Messages")
        status_term_layout = QVBoxLayout()
        status_term_layout.addWidget(self.statusTerminal)
        self.statusTermGroupBox.setMinimumHeight(100)

        self.statusTermGroupBox.setLayout(status_term_layout)
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.controls_group)  
        bottom_layout.addWidget(self.statusTermGroupBox)

        bottom_container = QWidget()
        bottom_container.setLayout(bottom_layout)
        vertSplitter.addWidget(bottom_container)
        split_screen_layout = QVBoxLayout()
        # split_screen_layout.addWidget(splitter)
        split_screen_layout.addWidget(vertSplitter)
        # split_screen_layout.addWidget(self.statusTermGroupBox) 
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
        self.gear_slider_splitter.setValue(value)

    def pub_next_state(self):
        self.next_state_pub.publish(True)

    def pub_manual_abort(self):
        self.manual_abort_pub.publish(True)

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

        self.cameras = ["Zed (front) camera", "Butt camera"]
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