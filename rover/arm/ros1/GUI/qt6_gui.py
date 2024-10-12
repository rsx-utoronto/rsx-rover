from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QLineEdit, QComboBox, QSlider, QGridLayout, QGroupBox, QSpinBox
)
from PyQt6.QtCore import Qt
import sys

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface")
        self.setGeometry(100, 100, 800, 600)
        self.initUI()
    def button_is_clicked(self):
        print("Clicked")

    def initUI(self):
        # Main Layout
        main_layout = QHBoxLayout(self)

        # Coordinates Control Section:w
        coordinates_group = QGroupBox("Inverse Kinematics: End Effector Coordinates and Rotation")
        coord_layout = QGridLayout()

        # Arrow Buttons for movement
        directions = ["Up", "Down", "Left", "Right", "Forward", "Backward",
                      "Rx", "Ry", "Rz", "-Rx", "-Ry", "-Rz"]
        arrow_buttons = {d: QPushButton(d) for d in directions}

        for direction, button in arrow_buttons.items():
            button.clicked.connect(self.button_is_clicked)

        coord_layout.addWidget(arrow_buttons["Up"], 1, 1)
        coord_layout.addWidget(arrow_buttons["Down"], 3, 1)
        coord_layout.addWidget(arrow_buttons["Left"], 2, 0)
        coord_layout.addWidget(arrow_buttons["Right"], 2, 2)
        coord_layout.addWidget(arrow_buttons["Forward"], 0, 0)
        coord_layout.addWidget(arrow_buttons["Backward"], 0, 2)

        # Rotation Controls
        coord_layout.addWidget(arrow_buttons["Rx"], 4, 0)
        coord_layout.addWidget(arrow_buttons["-Rx"], 4, 2)
        coord_layout.addWidget(arrow_buttons["Ry"], 5, 0)
        coord_layout.addWidget(arrow_buttons["-Ry"], 5, 2)
        coord_layout.addWidget(arrow_buttons["Rz"], 6, 0)
        coord_layout.addWidget(arrow_buttons["-Rz"], 6, 2)

        coordinates_group.setLayout(coord_layout)

        # 3D View Section
        view_group = QGroupBox("Camera View")
        view_layout = QVBoxLayout()

        self.user_coord_box = QComboBox()
        self.user_coord_box.addItems(["Camera View 1", "Camera View 2"])
        view_layout.addWidget(self.user_coord_box)

        view_layout.addStretch(1)
        
        self.coord_view_label = QLabel("Cameras Goes Here")
        self.coord_view_label.setFixedHeight(200)
        view_layout.addWidget(self.coord_view_label)



        # Modules for Science Team

        science_modules = QGroupBox("Science Modules")
        science_layout = QVBoxLayout()

        self.module1 = QPushButton("Module 1")
        self.module2 = QPushButton("Module 2")
        self.module3 = QPushButton("Module 3")
        self.module4 = QPushButton("Module 4")

        science_layout.addWidget(self.module1)
        science_layout.addWidget(self.module2)
        science_layout.addWidget(self.module3)
        science_layout.addWidget(self.module4)
        module_types = ["Module 1", "Module 2", "Module 3", "Module 4"]
        module_buttons = {m: QPushButton(m) for m in module_types}

        science_layout.addWidget(module_buttons["Module 1"])
        science_layout.addWidget(module_buttons["Module 2"])
        science_layout.addWidget(module_buttons["Module 3"])
        science_layout.addWidget(module_buttons["Module 4"])

        for modules, button in module_buttons.items():
            button.clicked.connect(self.button_is_clicked)

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        view_layout.addWidget(self.speed_slider)

        science_modules.setLayout(science_layout)

        view_group.setLayout(view_layout)

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
        self.y_coord = QLineEdit("Y: 0.0 mm")
        self.z_coord = QLineEdit("Z: 0.0 mm")

        coords_layout.addWidget(self.x_coord)
        coords_layout.addWidget(self.y_coord)
        coords_layout.addWidget(self.z_coord)

        # Rotation Information
        self.rx_coord = QLineEdit("Rx: 0.0°")
        self.ry_coord = QLineEdit("Ry: 0.0°")
        self.rz_coord = QLineEdit("Rz: 0.0°")

        coords_layout.addWidget(self.rx_coord)
        coords_layout.addWidget(self.ry_coord)
        coords_layout.addWidget(self.rz_coord)

        coords_group.setLayout(coords_layout)

        modes_group = QGroupBox("Modes")
        modes_layout = QGridLayout()

        self.idle_mode = QPushButton("Idle")
        self.setup_mode = QPushButton("Setup")
        self.manual_mode = QPushButton("Manual")
        self.inverse_kin_mode = QPushButton("Inverse Kin")
        self.dig_mode = QPushButton("Dig")
        self.pickup_mode = QPushButton("Pickup")
        self.custom1_mode = QPushButton("Custom 1")
        self.custom2_mode = QPushButton("Custom 2")

        modes_layout.addWidget(self.idle_mode, 0, 0)
        modes_layout.addWidget(self.setup_mode, 0, 1)
        modes_layout.addWidget(self.manual_mode, 1, 0)
        modes_layout.addWidget(self.inverse_kin_mode, 1, 1)
        modes_layout.addWidget(self.dig_mode, 2, 0)
        modes_layout.addWidget(self.pickup_mode, 2, 1)
        modes_layout.addWidget(self.custom1_mode, 3, 0)
        modes_layout.addWidget(self.custom2_mode, 3, 1)

        modes_group.setLayout(modes_layout)

        coords_layout.addWidget(modes_group)
        coords_group.setLayout(coord_layout)

        coords_layout.addWidget(science_modules)

        # Adding sections to the main layout
        main_layout.addWidget(coordinates_group, stretch=1)
        main_layout.addWidget(view_group, stretch=3)
        main_layout.addWidget(joints_group, stretch=1)
        main_layout.addWidget(coords_group, stretch=1)

        # Bottom Buttons
        bottom_layout = QHBoxLayout()
        self.move_home = QPushButton("Move to Home")
        self.move_origin = QPushButton("Move to Origin")
        self.freemove = QPushButton("Freemove")

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
