#! /usr/bin/env python3

from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QComboBox, QSlider, QGridLayout, QGroupBox, QSpinBox
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt
import sys

from gui_camera import ROSVideoSubscriber # Imports video display functionality
from arm_gui_controller import GuiControllerNode  # Class for sending commands to manipulator

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface")
        self.setGeometry(100, 100, 800, 600)
        self.camera_topic_name = ["/camera/color/image_king", "/camera/color/image_raw"]
        self.initUI()

        self.controller = GuiControllerNode()  # Initialize controller
        # self.update_current_position() # DOESN"T WORK

    
    def button_is_clicked(self,command):
        # self.controller.on_press(command)  # Send command to controller
        # self.controller.on_release()       # Reset
        print(command)

    def update_current_position(self):
        # value = 0
        # while True:
        #     self.x_coord.setText(f"X: {value:.1f} mm")
        #     self.y_coord.setText(f"Y: {value:.1f} mm")
        #     self.z_coord.setText(f"Z: {value:.1f} mm")
        #     self.rx_coord.setText(f"Rx: {value:.1f}°") 
        #     self.ry_coord.setText(f"Ry: {value:.1f}°") 
        #     self.rz_coord.setText(f"Rz: {value:.1f}°")
        #     value += 1
        pass

    def update_image(self, qt_image):
        # Clear old pixmap
        self.coord_view_label.clear()
        # Update the QLabel with the latest frame
        pixmap = QPixmap.fromImage(qt_image)
        self.coord_view_label.setPixmap(pixmap)
        self.coord_view_label.setScaledContents(True)  # Optional: scales the image to fit the label size

    def on_view_selected(self, index):
        self.video_subscriber.sub.unregister()
        self.video_subscriber = ROSVideoSubscriber(self.camera_topic_name[index])
        self.video_subscriber.frame_received.connect(self.update_image)

    def initUI(self):
        # Main Layout
        main_layout = QHBoxLayout()

        # Coordinates Control Section:w
        coordinates_group = QGroupBox("Inverse Kinematics: End Effector Coordinates and Rotation")
        coord_layout = QGridLayout()

        # Arrow Buttons for movement
        directions = ["Up", "Down", "Left", "Right", "Forward", "Backward",
                      "Rx", "Ry", "Rz", "-Rx", "-Ry", "-Rz","Open Grip", "Close Grip"]
        arrow_buttons = {d: QPushButton(d) for d in directions}

        for direction, button in arrow_buttons.items():
            button.clicked.connect(lambda _, d=direction: self.button_is_clicked(d))
            
        coord_layout.addWidget(arrow_buttons["Up"], 1, 1)
        coord_layout.addWidget(arrow_buttons["Down"], 3, 1)
        coord_layout.addWidget(arrow_buttons["Left"], 2, 0)
        coord_layout.addWidget(arrow_buttons["Right"], 2, 2)
        coord_layout.addWidget(arrow_buttons["Forward"], 0, 0)
        coord_layout.addWidget(arrow_buttons["Backward"], 0, 2)

        # Rotation Controls
        coord_layout.addWidget(arrow_buttons["Rx"], 4, 0)
        coord_layout.addWidget(arrow_buttons["-Rx"], 4, 1)
        coord_layout.addWidget(arrow_buttons["Ry"], 5, 0)
        coord_layout.addWidget(arrow_buttons["-Ry"], 5, 1)
        coord_layout.addWidget(arrow_buttons["Rz"], 6, 0)
        coord_layout.addWidget(arrow_buttons["-Rz"], 6, 1)

        # Grip Control
        coord_layout.addWidget(arrow_buttons["Open Grip"], 5, 2)
        coord_layout.addWidget(arrow_buttons["Close Grip"], 6, 2)
        
        coordinates_group.setLayout(coord_layout)

        # 3D View Section
        view_group = QGroupBox("Camera View")
        view_layout = QVBoxLayout()

        self.camera_view_box  = QComboBox()
        self.camera_view_box .addItems(["Camera View 1", "Camera View 2"])
        view_layout.addWidget(self.camera_view_box )

        view_layout.addStretch(1)
        
        self.coord_view_label = QLabel("Cameras Goes Here")
        self.coord_view_label.setFixedHeight(480)
        self.coord_view_label.setFixedWidth(640)
        view_layout.addWidget(self.coord_view_label)

        # Initialize the ROS video subscriber
        self.video_subscriber = ROSVideoSubscriber(self.camera_topic_name[0])
        self.video_subscriber.frame_received.connect(self.update_image)

        self.camera_view_box.currentIndexChanged.connect(self.on_view_selected)
        # -------------------------------- _

        # Arm Power Options
        power_group = QGroupBox("Arm Power")
        power_layout = QHBoxLayout()

        power_layout.setContentsMargins(0,0,0,0)

        self.power_on = QPushButton("ON")
        self.power_on.clicked.connect(lambda _: self.button_is_clicked("ON"))         
        self.power_off = QPushButton("OFF")
        self.power_off.clicked.connect(lambda _: self.button_is_clicked("OFF"))         
        self.power_reset = QPushButton("Reset")
        self.power_reset.clicked.connect(lambda _: self.button_is_clicked("Reset"))         
        
        power_group.setStyleSheet('QPushButton {font-size: 20px}')
        self.power_on.setStyleSheet('QPushButton {background-color: #00FF00; color: #000000}')
        self.power_off.setStyleSheet('QPushButton {background-color: #FF0000}')

        power_layout.addWidget(self.power_on)
        power_layout.addWidget(self.power_off)
        power_layout.addWidget(self.power_reset)

        power_group.setLayout(power_layout)
        view_layout.addWidget(power_group)
        view_group.setLayout(view_layout)

        # Modules for Science Team
        module_types = ["Module 1", "Module 2", "Module 3", "Module 4"]
        module_buttons = {m: QPushButton(m) for m in module_types}

        science_modules = QGroupBox("Science Modules")
        science_layout = QVBoxLayout()

        science_layout.addWidget(module_buttons["Module 1"])
        science_layout.addWidget(module_buttons["Module 2"])
        science_layout.addWidget(module_buttons["Module 3"])
        science_layout.addWidget(module_buttons["Module 4"])

        for modules, button in module_buttons.items():
            button.clicked.connect(lambda _, m=modules: self.button_is_clicked(m))
            
        science_modules.setLayout(science_layout)


        # Joints Control Section
        joints_group = QGroupBox("Joints Control")
        joints_layout = QVBoxLayout()

        self.joint_controls = []
        for i in range(6):
            joint_control = QHBoxLayout()
            joint_label = QLabel(f"Joint {i + 1}")
            joint_spin = QSpinBox()
            joint_spin.setRange(-180, 180)

            joint_control.addWidget(joint_label)
            joint_control.addWidget(joint_spin)
            joints_layout.addLayout(joint_control)

        joints_group.setLayout(joints_layout)

        # Coordinates Section (with Rotation Information)
        coords_group = QGroupBox("End Effector Coordinates")
        coords_layout = QVBoxLayout()

        # Position Information
        self.x_coord = QLineEdit("X: 0.0 mm")
        self.x_coord.setReadOnly(True)
        self.y_coord = QLineEdit("Y: 0.0 mm")
        self.y_coord.setReadOnly(True)
        self.z_coord = QLineEdit("Z: 0.0 mm")
        self.z_coord.setReadOnly(True)

        coords_layout.addWidget(self.x_coord)
        coords_layout.addWidget(self.y_coord)
        coords_layout.addWidget(self.z_coord)

        # Rotation Information
        self.rx_coord = QLineEdit("Rx: 0.0°")
        self.rx_coord.setReadOnly(True)
        self.ry_coord = QLineEdit("Ry: 0.0°")
        self.ry_coord.setReadOnly(True)
        self.rz_coord = QLineEdit("Rz: 0.0°")
        self.rz_coord.setReadOnly(True)

        coords_layout.addWidget(self.rx_coord)
        coords_layout.addWidget(self.ry_coord)
        coords_layout.addWidget(self.rz_coord)

        coords_group.setLayout(coords_layout)

        modes_group = QGroupBox("Modes")
        modes_layout = QGridLayout()

        mode_types = ["Idle", "Setup", "Manual", "Inverse Kin", "Dig", "Pick up", "Custom 1", "Custom 2"]
        mode_buttons = {m: QPushButton(m) for m in mode_types}

        modes_layout.addWidget(mode_buttons["Idle"], 0, 0)
        modes_layout.addWidget(mode_buttons["Setup"], 0, 1)
        modes_layout.addWidget(mode_buttons["Manual"], 1, 0)
        modes_layout.addWidget(mode_buttons["Inverse Kin"], 1, 1)
        modes_layout.addWidget(mode_buttons["Dig"], 2, 0)
        modes_layout.addWidget(mode_buttons["Pick up"], 2, 1)
        modes_layout.addWidget(mode_buttons["Custom 1"], 3, 0)
        modes_layout.addWidget(mode_buttons["Custom 2"], 3, 1)

        for modes, button in mode_buttons.items():
            button.clicked.connect(lambda _, m=modes: self.button_is_clicked(m))
            
        modes_group.setLayout(modes_layout)

        coords_layout.addWidget(modes_group)
        coords_layout.addWidget(science_modules)
        coords_group.setLayout(coords_layout)


        # Adding sections to the main layout
        main_layout.addWidget(coordinates_group)
        main_layout.addWidget(view_group, stretch=3)
        main_layout.addWidget(joints_group)
        main_layout.addWidget(coords_group)

        # Bottom Buttons
        bottom_layout = QHBoxLayout()
        self.move_home = QPushButton("Move to Home")
        self.move_home.clicked.connect(lambda _: self.button_is_clicked("Move to Home"))         
        self.move_origin = QPushButton("Move to Origin")
        self.move_origin.clicked.connect(lambda _: self.button_is_clicked("Move to Origin"))         
        self.freemove = QPushButton("Freemove")
        self.freemove.clicked.connect(lambda _: self.button_is_clicked("Freemove"))         

        bottom_layout.addWidget(self.move_home)
        bottom_layout.addWidget(self.move_origin)
        bottom_layout.addWidget(self.freemove)

        # Main Vertical Layout
        main_vertical_layout = QVBoxLayout(self)
        main_vertical_layout.addLayout(main_layout)
        main_vertical_layout.addLayout(bottom_layout)
        self.setLayout(main_vertical_layout)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlGUI()
    window.show()
    sys.exit(app.exec())
