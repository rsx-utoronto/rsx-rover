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
    QListWidget, QListWidgetItem
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, CompressedImage
from std_msgs.msg import Float32MultiArray, String, Bool
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
        rospy.Subscriber('/status', String, self.string_callback)
        self.received_strings = []
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

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.string_list)
        self.setLayout(layout)
    
    def string_callback(self, msg):
        self.update_status_signal.emit(msg.data.strip())  

    def update_string_list(self, new_string):
        self.received_strings.append(new_string)
        self.string_list.setPlainText("\n".join(self.received_strings))
        self.string_list.moveCursor(QTextCursor.End)

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
        coordArray =["GNSS 1","GNSS 2", "AR 1", "AR 2", "AR 3", "OBJ 1", "OBJ 2"]
        for i in range(7):  # Example: 5 items in dropdown
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
    def __init__(self):
        super().__init__()
        self.longLat_pub = rospy.Publisher('/long_lat_goal_array', Float32MultiArray, queue_size=5)
        self.array = Float32MultiArray()
        layout = QVBoxLayout(self)

        self.combo = EditableComboBox()
        layout.addWidget(self.combo)

        # Add button to collect data
        self.submit_button = QPushButton("Get Data")
        self.submit_button.clicked.connect(self.collect_data)
        layout.addWidget(self.submit_button)

        self.setLayout(layout)

    def collect_data(self):
        data = self.combo.get_all_data()
        print("Collected Data:")
        self.array.data = data
        self.longLat_pub.publish(self.array)
        for item in data:
            print(item)  # Prints each row's values




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

#camera feed that displays one camera at a time (use switch_camera)
# Update the CameraFeed class to handle multiple labels
class CameraFeed:
    def __init__(self, label1, label2, splitter):
        self.bridge = CvBridge()
        self.image_sub1 = None 
        self.image_sub2 = None 
        self.label1 = label1
        self.label2 = label2
        self.splitter = splitter
        self.active_cameras = {"Zed (front) camera": False, "Butt camera": False}

    def register_subscriber1(self):
        if self.image_sub1 is None:  # Only register if not already registered
            self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color/compressed", CompressedImage, self.callback1)

    def unregister_subscriber1(self):
        if self.image_sub1:
            self.image_sub1.unregister()
            self.image_sub1 = None

    def register_subscriber2(self):
        if self.image_sub2 is None:  # Only register if not already registered
            self.image_sub2 = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback2)

    def unregister_subscriber2(self):
        if self.image_sub2:
            self.image_sub2.unregister()
            self.image_sub2 = None

    def callback1(self, data):
        if self.active_cameras["Zed (front) camera"]:
            self.update_image(data, self.label1)

    def callback2(self, data):
        if self.active_cameras["Butt camera"]:
            self.update_image(data, self.label2)

    def update_image(self, data, label):
        np_arr = np.frombuffer(data.data, np.uint8)  # Convert compressed data to NumPy array
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qimg = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        label.setPixmap(pixmap)

    def update_active_cameras(self, active_cameras):
        self.active_cameras = active_cameras
        self.update_subscribers()
        self.update_visibility()

    def update_subscribers(self):
        """Update the ROS subscribers based on active cameras."""
        if self.active_cameras["Zed (front) camera"]:
            self.register_subscriber1()
        else:
            self.unregister_subscriber1()

        if self.active_cameras["Butt camera"]:
            self.register_subscriber2()
        else:
            self.unregister_subscriber2()

    def update_visibility(self):
        """Update the visibility of camera labels based on active cameras."""
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
        else:  # Both active
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

        # Create tab
        self.split_screen_tab = QWidget()
        self.longlat_tab = QWidget()
        self.controlTab = QWidget()

        # Add tab to QTabWidget
        self.tabs.addTab(self.split_screen_tab, "Main Gui")
        self.tabs.addTab(self.longlat_tab, "State Machine")
        self.tabs.addTab(self.controlTab, "Controls")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)

        self.setup_split_screen_tab()
        self.setup_lngLat_tab()
        self.setup_control_tab()



        
    #unused utility: if multiple tabs used can have triggers when tab sswitched
    def on_tab_changed(self, index):
        if index == 1:  # Map Tab
            print("map tab")  
        elif index == 2:  # Split Screen Tab
            print("split tab") # Show map viewer in split screen tab

    def setup_control_tab(self):
        # Controls section
        controls_group = QGroupBox("Controls")
        controls_layout = QHBoxLayout()

        # Joystick
        self.joystick_splitter = Joystick(self.velocity_control)
        joystick_group = QGroupBox("Joystick")
        joystick_layout = QVBoxLayout()
        joystick_layout.addWidget(self.joystick_splitter)
        joystick_group.setLayout(joystick_layout)

        # Gear slider
        gear_group = QGroupBox("Gear Control")
        slider_layout = QVBoxLayout()
        self.gear_slider_splitter = QSlider(Qt.Horizontal, self.split_screen_tab)
        self.gear_slider_splitter.setRange(1, 10)
        self.gear_slider_splitter.setTickPosition(QSlider.TicksBelow)
        self.gear_slider_splitter.setTickInterval(1)
        self.gear_slider_splitter.valueChanged.connect(self.change_gear)
        slider_layout.addWidget(self.gear_slider_splitter)
        gear_group.setLayout(slider_layout)

        # Add joystick and gear controls side by side
        controls_layout.addWidget(joystick_group)
        controls_layout.addWidget(gear_group)
        controls_group.setLayout(controls_layout)
        # vertical_splitter.addWidget(controls_group)
        control_tab_layout = QVBoxLayout()
        control_tab_layout.addWidget(controls_group)
        self.controlTab.setLayout(control_tab_layout)

    def setup_lngLat_tab(self):
        self.lngLatEntry = LngLatEntryBar()
        self.stateMachineDialog = StateMachineStatus()
        self.arucoBox = ArucoWidget()
        self.statusTerminal = statusTerminal()

        # Create a group box for the ArucoWidget
        self.arucoGroupBox = QGroupBox("Aruco Tags")
        aruco_layout = QVBoxLayout()
        aruco_layout.addWidget(self.arucoBox)
        self.arucoGroupBox.setLayout(aruco_layout)

        # Create a group box for the status terminal
        self.statusTermGroupBox = QGroupBox("Status Messages")
        status_term_layout = QVBoxLayout()
        status_term_layout.addWidget(self.statusTerminal)
        self.statusTermGroupBox.setLayout(status_term_layout)

        # Main layout for the tab
        Lnglat_tab_layout = QVBoxLayout()
        Lnglat_tab_layout.addWidget(self.lngLatEntry)
        Lnglat_tab_layout.addWidget(self.stateMachineDialog)
        Lnglat_tab_layout.addWidget(self.arucoGroupBox) 
        Lnglat_tab_layout.addWidget(self.statusTermGroupBox) 

        self.longlat_tab.setLayout(Lnglat_tab_layout)
    #used to initialize main tab with splitters
    def setup_split_screen_tab(self):
        splitter = QSplitter(Qt.Horizontal)
        # Add camera feed to the splitter
        camera_group = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout()
        

        self.camera_label1 = QLabel()
        self.camera_label1.setMinimumSize(320, 240)
        self.camera_label1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        

        self.camera_label2 = QLabel()
        self.camera_label2.setMinimumSize(320, 240)
        self.camera_label2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        # ROS functionality
        self.camerasplitter = QSplitter(Qt.Horizontal)
        self.camera_feed = CameraFeed(self.camera_label1, self.camera_label2,self.camerasplitter)
        

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
        self.checkbox_setting_splitter = QCheckBox("Recentre map when rover offscreen")
        self.checkbox_setting_splitter.setChecked(False)  # Set the default state to unchecked
        # self.checkbox_setting_splitter.stateChanged.connect(self.on_checkbox_state_changed)
        self.checkbox_setting_splitter.stateChanged.connect(
            lambda state: self.on_checkbox_state_changed(state, self.map_overlay_splitter)
        )
        map_layout.addWidget(self.checkbox_setting_splitter)
        map_layout.addWidget(self.map_overlay_splitter)
        map_group.setLayout(map_layout)

        # Add widgets to the splitter
        splitter.addWidget(camera_group)
        splitter.addWidget(map_group)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        
        
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
        self.velocity_control.set_gear(value)
        print(f"Changed to Gear: {value}")

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)

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
