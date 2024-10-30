#!/usr/bin/env python3

import sys
from enum import Enum
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QComboBox, QGridLayout, QSlider, QHBoxLayout, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView  # Import QWebEngineView for the map
from PyQt5.QtCore import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter
import plotly.express as px
import os

class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class VelocityControl:
    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=10)
        self.gear = 1  # Initialize gear to Low (1 by default)

    def set_gear(self, gear):
        """Sets the current gear level (1-10 levels)"""
        self.gear = gear
        print(f"Gear set to: {gear}")  # Debug print

    def send_velocity(self, linear_x, angular_z):
        """Scale velocities according to the current gear"""
        # Adjust velocities based on the gear
        linear_x *= (self.gear/10)*2.5  # Scale linear velocity
        angular_z *= (self.gear/10)*2.5  # Scale angular velocity

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)
        print(f"Publishing to /drive: linear_x = {linear_x}, angular_z = {angular_z}, gear = {self.gear}")  # Debug print

# No change in the Joystick class, it already sends the velocities to VelocityControl

class Direction: 
    def __init__(self):
        self.LinX = 0
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

        # Gear slider
        self.gear_slider = QSlider(Qt.Vertical)
        self.gear_slider.setRange(1, 10)  # 1-10 gear options
        self.gear_slider.setTickPosition(QSlider.TicksBelow)
        self.gear_slider.setTickInterval(1)
        self.gear_slider.setValue(1)  # Set default value to Low Gear
        self.gear_slider.valueChanged.connect(self.change_gear)  # Connect slider value change to gear change
        
        # Create a layout for the slider and labels
        slider_layout = QHBoxLayout()  # Horizontal layout for labels and slider

        # Create labels for each tick on the slider
        label_layout = QVBoxLayout()  # Vertical layout for labels
        for i in range(10, 0, -1):  # Create labels from 10 to 1
            label = QLabel(str(i))
            label.setAlignment(Qt.AlignCenter)  # Center align the labels
            label_layout.addWidget(label)

        # Create a layout for the gear label and the slider
        gear_label_layout = QVBoxLayout()  # Vertical layout for the gear label and the slider
        gear_label = QLabel("Gear")
        gear_label.setAlignment(Qt.AlignCenter)  # Center align the gear label
        gear_label_layout.addWidget(gear_label)  # Add the gear label
        gear_label_layout.addWidget(self.gear_slider)  # Add the slider below the gear label

        # Add both layouts to the horizontal layout
        slider_layout.addLayout(label_layout)  # Add labels to the left
        slider_layout.addLayout(gear_label_layout)  # Add gear label and slider to the right

        # Layout setup
        layout = QGridLayout()
        layout.addWidget(QLabel("Camera Feed"), 0, 0)
        layout.addWidget(self.camera_label, 1, 0, 1, 2)
        layout.addWidget(QLabel("Select Camera"), 2, 0)
        layout.addWidget(self.camera_selector, 2, 1)
        layout.addLayout(slider_layout, 1, 1, 1, 1)  # Adjusted position for slider layout
        layout.addWidget(self.joystick, 3, 0, 1, 2)

        # Add the map widget
        self.map_view = QWebEngineView(self)
        self.init_map()
        layout.addWidget(self.map_view, 4, 0, 1, 2)  # Add map below other widgets

        self.setLayout(layout)

        # Start ROS in a separate thread
        self.ros_thread = ROSWorker()
        self.ros_thread.start()

    def change_gear(self, value):
        """Change the gear based on the value from the QSlider"""
        gear = value  # The value from the slider directly corresponds to the gear
        self.velocity_control.set_gear(gear)
        print(f"Changed to Gear: {gear}")  # Debug print

    def switch_camera(self, index):
        self.camera_feed.switch_camera(index + 1)

    def init_map(self):
        #Initialize the Mapbox map and set it to the map view
        # Mapbox access token (replace with your own)
        mapbox_access_token = 'pk.eyJ1IjoibmJ5bGltIiwiYSI6ImNsejdkNXJybDAzNHEyaXBvYTg5MHM4dzcifQ.j6GwNwJW9bImvDU_fA-R0A'

        # Create a simple Plotly map
        fig = px.scatter_mapbox(lat=[37.7749], lon=[-122.4194], hover_name=["San Francisco"], zoom=10)
        fig.update_layout(mapbox_style="open-street-map",
                        mapbox_accesstoken=mapbox_access_token,
                        mapbox_zoom=10,
                        mapbox_center={"lat": 37.7749, "lon": -122.4194})

        # Generate the HTML string for the map
        html_string = fig.to_html(full_html=True, include_plotlyjs='cdn')

        # Load the HTML string into the QWebEngineView
        self.map_view.setHtml(html_string)



if __name__ == '__main__':
    rospy.init_node('rover_gui', anonymous=False)  # Initialize ROS node
    app = QApplication(sys.argv)
    gui = RoverGUI()
    gui.show()
    sys.exit(app.exec_())
