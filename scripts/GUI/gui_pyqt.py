#!/usr/bin/env python3


import sys
import rospy
import map_viewer as map_viewer
from pathlib import Path


from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QGridLayout, \
    QSlider, QHBoxLayout, QVBoxLayout, QMainWindow, QTabWidget, QGroupBox, QFrame, \
    QCheckBox,QSplitter,QSizePolicy,QStylePainter, QStyleOptionComboBox, QStyle, \
    QToolButton, QMenu, QLineEdit, QFormLayout, QPushButton
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QPointF
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix  
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter,QPalette,QStandardItemModel

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

#type bars widget for latitude longitude entry
class LngLatEntryBar(QWidget):
    def __init__(self):
        super().__init__()
        self.longLat_pub = rospy.Publisher('/long_lat_goal_array', Float32MultiArray)
        self.longitudeBar = QLineEdit()
        self.latitudeBar = QLineEdit()
        self.completeLong = False
        self.completeLat = False
        self.array = Float32MultiArray()
        self.array.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] #adapt size for 
        self.array_index = 0
        flo = QFormLayout()
        flo.addRow("Longitude", self.longitudeBar)
        flo.addRow("Latitude", self.latitudeBar)

        # Create the button and connect it to the method
        self.sendButton = QPushButton("Send Coordinates")
        self.sendButton.clicked.connect(self.check_and_send_coordinates)

        # Add button to the layout
        flo.addWidget(self.sendButton)

        self.setLayout(flo)

    def longitudeEntry(self):
        self.completeLong = True
        if self.completeLat:
            self.processCoordinates()

    def latitudeEntry(self):
        self.completeLat = True
        if self.completeLong:
            self.processCoordinates()

    def processCoordinates(self):
        """
        Process the coordinates: fetch values, publish them, and reset the input fields.
        """
        longitude = self.longitudeBar.text()
        latitude = self.latitudeBar.text()
        self.sendCoordinates(longitude, latitude)
        self.resetCoordinates()

    def check_and_send_coordinates(self):
        """
        Checks if both longitude and latitude have values before sending.
        """
        longitude = self.longitudeBar.text()
        latitude = self.latitudeBar.text()
        if longitude and latitude:
            self.sendCoordinates(longitude, latitude)
            self.resetCoordinates()  # Optionally reset fields after sending
        else:
            print("Please enter both Longitude and Latitude.")

    def resetCoordinates(self):
        """
        Reset the state of longitude and latitude completion flags and clear the input fields.
        """
        self.completeLong = False
        self.completeLat = False
        self.longitudeBar.clear()
        self.latitudeBar.clear()

    def sendCoordinates(self, longitude, latitude):
        """
        Send or print the coordinates.
        """
        self.array.data[self.array_index] = longitude
        self.array.data[self.array_index +1] = latitude
        self.array_index +=2
        self.longLat_pub.publish(self.array.data)
        print(f"Publish coordinates: Longitude = {longitude}, Latitude = {latitude}")



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
        print(f"Publishing to /drive: linear_x = {linear_x}, angular_z = {angular_z}, gear = {self.gear}")

#joystick that controlss velocity magnitude and direction
class Joystick(QWidget):
    def __init__(self, velocity_control, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(200, 200)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 100
        self.direction = Direction()
        self.velocity_control = VelocityControl()

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
            self.image_sub1 = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback1)

    def unregister_subscriber1(self):
        if self.image_sub1:
            self.image_sub1.unregister()
            self.image_sub1 = None

    def register_subscriber2(self):
        if self.image_sub2 is None:  # Only register if not already registered
            self.image_sub2 = rospy.Subscriber("/camera/color/image_raw", Image, self.callback2)

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
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
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

        # Add tab to QTabWidget
        self.tabs.addTab(self.split_screen_tab, "Main Gui")
        self.tabs.addTab(self.longlat_tab, "Long Lat")

        # Connect tab change event
        self.tabs.currentChanged.connect(self.on_tab_changed)

        self.setup_split_screen_tab()
        self.setup_lngLat_tab()


        
    #unused utility: if multiple tabs used can have triggers when tab sswitched
    def on_tab_changed(self, index):
        if index == 1:  # Map Tab
            print("map tab")  
        elif index == 2:  # Split Screen Tab
            print("split tab") # Show map viewer in split screen tab
    def setup_lngLat_tab(self):
        self.lngLatEntry = LngLatEntryBar()
        Lnglat_tab_layout = QVBoxLayout()
        Lnglat_tab_layout.addWidget(self.lngLatEntry)
        self.longlat_tab.setLayout(Lnglat_tab_layout)
    #used to initialize main tab with splitters
    def setup_split_screen_tab(self):
        splitter = QSplitter(Qt.Horizontal)
        vertical_splitter = QSplitter(Qt.Vertical)
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
        vertical_splitter.addWidget(splitter)
        vertical_splitter.addWidget(controls_group)
        split_screen_layout = QVBoxLayout()
        split_screen_layout.addWidget(vertical_splitter)
        

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
