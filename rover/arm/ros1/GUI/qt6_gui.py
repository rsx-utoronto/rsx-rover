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

    def initUI(self):
        # Main Layout
        main_layout = QHBoxLayout(self)

        # Coordinates Control Section
        coordinates_group = QGroupBox("Inverse Kinematics: End Effector Coordinates and Rotation")
        coord_layout = QGridLayout()

        # Arrow Buttons for movement
        directions = ["Up", "Down", "Left", "Right", "Forward", "Backward",
                      "Rx", "Ry", "Rz", "-Rx", "-Ry", "-Rz"]
        arrow_buttons = {d: QPushButton(d) for d in directions}

        coord_layout.addWidget(arrow_buttons["Up"], 0, 1)
        coord_layout.addWidget(arrow_buttons["Down"], 2, 1)
        coord_layout.addWidget(arrow_buttons["Left"], 1, 0)
        coord_layout.addWidget(arrow_buttons["Right"], 1, 2)
        coord_layout.addWidget(arrow_buttons["Forward"], 1, 1)
        coord_layout.addWidget(arrow_buttons["Backward"], 1, 3)

        # Rotation Controls
        coord_layout.addWidget(arrow_buttons["Rx"], 3, 0)
        coord_layout.addWidget(arrow_buttons["-Rx"], 3, 1)
        coord_layout.addWidget(arrow_buttons["Ry"], 4, 0)
        coord_layout.addWidget(arrow_buttons["-Ry"], 4, 1)
        coord_layout.addWidget(arrow_buttons["Rz"], 5, 0)
        coord_layout.addWidget(arrow_buttons["-Rz"], 5, 1)

        coordinates_group.setLayout(coord_layout)

        # 3D View Section
        view_group = QGroupBox("3D View")
        view_layout = QVBoxLayout()

        self.coord_view_label = QLabel("3D Robot View Goes Here")
        self.coord_view_label.setFixedHeight(200)
        view_layout.addWidget(self.coord_view_label)

        self.user_coord_box = QComboBox()
        self.user_coord_box.addItems(["User Coordinates 1", "User Coordinates 2"])
        view_layout.addWidget(self.user_coord_box)

        self.continuous_motion = QPushButton("Continuous Motion")
        self.discrete_motion = QPushButton("Discrete Motion")

        motion_layout = QHBoxLayout()
        motion_layout.addWidget(self.continuous_motion)
        motion_layout.addWidget(self.discrete_motion)

        view_layout.addLayout(motion_layout)

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        view_layout.addWidget(self.speed_slider)

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

        # Adding sections to the main layout
        main_layout.addWidget(coordinates_group)
        main_layout.addWidget(view_group)
        main_layout.addWidget(joints_group)
        main_layout.addWidget(coords_group)

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
