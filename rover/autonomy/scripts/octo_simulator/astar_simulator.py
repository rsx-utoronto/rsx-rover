#!/usr/bin/env python3
"""
3D A* Obstacle Avoidance Simulator
===================================
A standalone simulator for testing A* pathfinding with obstacle avoidance.
Features:
- Interactive 3D visualization
- Click to place/remove obstacles
- Real-time path planning visualization
- Point cloud generation from obstacles
- Robot footprint visualization
- Adjustable parameters
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from queue import PriorityQueue
import math
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QDoubleSpinBox,
                             QGroupBox, QGridLayout, QComboBox, QCheckBox,
                             QSlider, QSpinBox, QMessageBox, QTabWidget, QScrollArea)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


class Obstacle:
    """Represents a 3D obstacle in the environment."""
    def __init__(self, position, size, shape='cube'):
        self.position = np.array(position)  # (x, y, z)
        self.size = np.array(size)  # (width, height, depth)
        self.shape = shape  # 'cube' or 'sphere'
    
    def generate_points(self, resolution=0.05):
        """Generate point cloud points for this obstacle."""
        points = []
        x, y, z = self.position
        w, h, d = self.size
        
        if self.shape == 'cube':
            # Generate points on the surface and inside the cube
            x_range = np.arange(x - w/2, x + w/2, resolution)
            y_range = np.arange(y - d/2, y + d/2, resolution)
            z_range = np.arange(z, z + h, resolution)
            
            for xi in x_range:
                for yi in y_range:
                    for zi in z_range:
                        points.append([xi, yi, zi])
        
        elif self.shape == 'sphere':
            # Generate points in a sphere
            radius = min(w, d, h) / 2
            samples = int((4/3 * np.pi * radius**3) / (resolution**3))
            samples = max(100, min(samples, 10000))  # Limit point count
            
            for _ in range(samples):
                # Random point in sphere
                theta = np.random.uniform(0, 2*np.pi)
                phi = np.random.uniform(0, np.pi)
                r = radius * np.cbrt(np.random.uniform(0, 1))
                
                xi = x + r * np.sin(phi) * np.cos(theta)
                yi = y + r * np.sin(phi) * np.sin(theta)
                zi = z + r * np.cos(phi)
                points.append([xi, yi, zi])
        
        return np.array(points)
    
    def get_vertices(self):
        """Get vertices for visualization."""
        x, y, z = self.position
        w, h, d = self.size
        
        if self.shape == 'cube':
            # Define 8 vertices of a cube
            vertices = [
                [x - w/2, y - d/2, z],
                [x + w/2, y - d/2, z],
                [x + w/2, y + d/2, z],
                [x - w/2, y + d/2, z],
                [x - w/2, y - d/2, z + h],
                [x + w/2, y - d/2, z + h],
                [x + w/2, y + d/2, z + h],
                [x - w/2, y + d/2, z + h],
            ]
            return vertices
        return []


class AStarPlanner:
    """A* path planner adapted from the ROS implementation."""
    
    def __init__(self, grid_resolution=0.1, grid_size=(200, 200)):
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.grid_origin = (
            -(grid_size[0] * grid_resolution) / 2,
            -(grid_size[1] * grid_resolution) / 2
        )
        
        self.occupancy_grid = None
        self.height_grid = None
        self.obstacle_threshold = 100
        self.z_min = -0.25
        self.z_max = 3.0
        
        # Robot footprint (rectangular)
        self.robot_corners = np.array([
            [0.3, 0.3],
            [0.3, -0.3],
            [-0.3, -0.3],
            [-0.3, 0.3]
        ])
    
    def update_grid_from_pointcloud(self, points):
        """Update occupancy grid from point cloud data."""
        w, h = self.grid_size
        new_height_grid = np.zeros((h, w), dtype=np.float32)
        grid = np.zeros((h, w), dtype=np.int8)
        
        for point in points:
            x, y, z = point
            
            # Filter by height threshold
            if not (self.z_min < z < self.z_max):
                continue
            
            gx, gy = self.world_to_grid(x, y)
            if 0 <= gx < w and 0 <= gy < h:
                grid[gy, gx] = self.obstacle_threshold
                new_height_grid[gy, gx] = max(new_height_grid[gy, gx], z)
        
        self.height_grid = new_height_grid
        self.occupancy_grid = grid
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        gx = int(round((x - self.grid_origin[0]) / self.grid_resolution))
        gy = int(round((y - self.grid_origin[1]) / self.grid_resolution))
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates."""
        x = gx * self.grid_resolution + self.grid_origin[0]
        y = gy * self.grid_resolution + self.grid_origin[1]
        return x, y
    
    def height_cost(self, current, neighbor):
        """Calculate cost between two grid cells based on height difference."""
        current_gx, current_gy = current
        neighbor_gx, neighbor_gy = neighbor
        
        # Check obstacles
        if self.occupancy_grid[current_gy, current_gx] >= self.obstacle_threshold:
            return float('inf')
        if self.occupancy_grid[neighbor_gy, neighbor_gx] >= self.obstacle_threshold:
            return float('inf')
        
        # Calculate height difference cost
        height_diff = abs(
            self.height_grid[current_gy, current_gx] - 
            self.height_grid[neighbor_gy, neighbor_gx]
        )
        return height_diff + 1  # +1 for base movement cost
    
    def heuristic(self, node, goal):
        """Euclidean distance heuristic."""
        return np.linalg.norm(np.array(node) - np.array(goal))
    
    def transform_corners(self, pose):
        """Transform robot corners to global frame."""
        x_pose, y_pose, theta = pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        transformed = []
        for corner in self.robot_corners:
            local_x, local_y = corner
            x = local_x * cos_theta - local_y * sin_theta + x_pose
            y = local_x * sin_theta + local_y * cos_theta + y_pose
            transformed.append((x, y))
        
        return transformed
    
    def is_pose_valid(self, pose):
        """Check if a pose is valid (robot footprint doesn't collide)."""
        for corner in self.transform_corners(pose):
            x, y = corner
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if (0 <= grid_x < self.occupancy_grid.shape[1] and 
                0 <= grid_y < self.occupancy_grid.shape[0]):
                if self.occupancy_grid[grid_y, grid_x] >= self.obstacle_threshold:
                    return False
        return True
    
    def get_neighbors(self, node, current_yaw=0):
        """Get valid neighbor nodes."""
        neighbors = []
        gx, gy = node
        
        # 8-connected grid
        for dx, dy in [(-1,0), (1,0), (0,1), (0,-1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            ngx = gx + dx
            ngy = gy + dy
            
            # Check bounds
            if not (0 <= ngx < self.grid_size[0] and 0 <= ngy < self.grid_size[1]):
                continue
            
            # Convert to world coordinates for footprint check
            nx, ny = self.grid_to_world(ngx, ngy)
            if self.is_pose_valid((nx, ny, current_yaw)):
                neighbors.append((ngx, ngy))
        
        return neighbors
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def a_star(self, start, goal, current_yaw=0):
        """A* pathfinding algorithm."""
        if self.occupancy_grid is None:
            return []
        
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        # Check if start and goal are valid
        if not self.is_pose_valid((start[0], start[1], current_yaw)):
            print("Start position is invalid!")
            return []
        if not self.is_pose_valid((goal[0], goal[1], current_yaw)):
            print("Goal position is invalid!")
            return []
        
        open_set = PriorityQueue()
        open_set.put((0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        closed = set()
        
        while not open_set.empty():
            _, current = open_set.get()
            
            if current in closed:
                continue
            closed.add(current)
            
            if current == goal_grid:
                path_grid = self.reconstruct_path(came_from, current)
                # Convert path to world coordinates
                path_world = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
                return path_world
            
            for neighbor in self.get_neighbors(current, current_yaw):
                if neighbor in closed:
                    continue
                
                cost = self.height_cost(current, neighbor)
                tentative_g = g_score[current] + cost
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    open_set.put((f_score[neighbor], neighbor))
        
        print("A* failed to find a path")
        return []


class AStarSimulator(QMainWindow):
    """Main simulator window with 3D visualization and controls."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("A* Obstacle Avoidance Simulator")
        self.setGeometry(100, 100, 1400, 900)
        
        # Simulation state
        self.obstacles = []
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0
        self.goal_pos = np.array([5.0, 5.0, 0.0])
        self.path = []
        self.planner = AStarPlanner(grid_resolution=0.1, grid_size=(200, 200))
        self.is_running = False
        self.path_index = 0
        
        # UI state
        self.selected_obstacle_idx = None
        self.obstacle_shape = 'cube'
        
        self.init_ui()
        self.update_visualization()
        
        # Timer for animation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        
    def init_ui(self):
        """Initialize the user interface."""
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left side: 3D visualization
        viz_widget = QWidget()
        viz_layout = QVBoxLayout(viz_widget)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        
        # Add navigation toolbar
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        viz_layout.addWidget(self.toolbar)
        viz_layout.addWidget(self.canvas)
        
        # Right side: Control panel
        control_widget = self.create_control_panel()
        
        # Add to main layout
        main_layout.addWidget(viz_widget, stretch=3)
        main_layout.addWidget(control_widget, stretch=1)
        
    def create_control_panel(self):
        """Create the control panel with all controls."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Title
        title = QLabel("Control Panel")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Create tabbed interface
        tab_widget = QTabWidget()
        tab_widget.addTab(self.create_positions_tab(), "🤖 Robot/Goal")
        tab_widget.addTab(self.create_obstacles_tab(), "🚧 Obstacles")
        tab_widget.addTab(self.create_simulation_tab(), "▶️ Simulation")
        tab_widget.addTab(self.create_camera_tab(), "📷 Camera")
        
        layout.addWidget(tab_widget)
        
        # Status label at bottom (always visible)
        self.status_label = QLabel("Ready")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 8px; border-radius: 3px; font-size: 11px; }")
        self.status_label.setMaximumHeight(80)
        layout.addWidget(self.status_label)
        
        return panel
    
    def create_positions_tab(self):
        """Create the robot and goal positions tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)
        
        # Robot position group
        robot_group = QGroupBox("Robot Position")
        robot_layout = QGridLayout()
        robot_layout.setSpacing(5)
        
        self.robot_x_spin = QDoubleSpinBox()
        self.robot_x_spin.setRange(-10, 10)
        self.robot_x_spin.setValue(0.0)
        self.robot_x_spin.setSingleStep(0.1)
        self.robot_x_spin.valueChanged.connect(self.on_robot_pos_changed)
        
        self.robot_y_spin = QDoubleSpinBox()
        self.robot_y_spin.setRange(-10, 10)
        self.robot_y_spin.setValue(0.0)
        self.robot_y_spin.setSingleStep(0.1)
        self.robot_y_spin.valueChanged.connect(self.on_robot_pos_changed)
        
        self.robot_yaw_spin = QDoubleSpinBox()
        self.robot_yaw_spin.setRange(-180, 180)
        self.robot_yaw_spin.setValue(0.0)
        self.robot_yaw_spin.setSingleStep(5.0)
        self.robot_yaw_spin.setSuffix("°")
        self.robot_yaw_spin.valueChanged.connect(self.on_robot_yaw_changed)
        
        robot_layout.addWidget(QLabel("X:"), 0, 0)
        robot_layout.addWidget(self.robot_x_spin, 0, 1)
        robot_layout.addWidget(QLabel("Y:"), 1, 0)
        robot_layout.addWidget(self.robot_y_spin, 1, 1)
        robot_layout.addWidget(QLabel("Yaw:"), 2, 0)
        robot_layout.addWidget(self.robot_yaw_spin, 2, 1)
        
        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)
        
        # Goal position group
        goal_group = QGroupBox("Goal Position")
        goal_layout = QGridLayout()
        goal_layout.setSpacing(5)
        
        self.goal_x_spin = QDoubleSpinBox()
        self.goal_x_spin.setRange(-10, 10)
        self.goal_x_spin.setValue(5.0)
        self.goal_x_spin.setSingleStep(0.1)
        self.goal_x_spin.valueChanged.connect(self.on_goal_pos_changed)
        
        self.goal_y_spin = QDoubleSpinBox()
        self.goal_y_spin.setRange(-10, 10)
        self.goal_y_spin.setValue(5.0)
        self.goal_y_spin.setSingleStep(0.1)
        self.goal_y_spin.valueChanged.connect(self.on_goal_pos_changed)
        
        goal_layout.addWidget(QLabel("X:"), 0, 0)
        goal_layout.addWidget(self.goal_x_spin, 0, 1)
        goal_layout.addWidget(QLabel("Y:"), 1, 0)
        goal_layout.addWidget(self.goal_y_spin, 1, 1)
        
        goal_group.setLayout(goal_layout)
        layout.addWidget(goal_group)
        
        # Quick actions
        quick_actions = QGroupBox("Quick Actions")
        quick_layout = QVBoxLayout()
        quick_layout.setSpacing(5)
        
        self.reset_btn = QPushButton("Reset Robot to Origin")
        self.reset_btn.clicked.connect(self.reset_robot)
        quick_layout.addWidget(self.reset_btn)
        
        quick_actions.setLayout(quick_layout)
        layout.addWidget(quick_actions)
        
        layout.addStretch()
        return tab
    
    def create_obstacles_tab(self):
        """Create the obstacles management tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)
        
        # Shape selection
        shape_group = QGroupBox("Obstacle Shape")
        shape_layout = QHBoxLayout()
        shape_layout.addWidget(QLabel("Shape:"))
        self.shape_combo = QComboBox()
        self.shape_combo.addItems(['cube', 'sphere'])
        self.shape_combo.currentTextChanged.connect(self.on_shape_changed)
        shape_layout.addWidget(self.shape_combo)
        shape_group.setLayout(shape_layout)
        layout.addWidget(shape_group)
        
        # Obstacle position controls
        pos_group = QGroupBox("Position")
        pos_layout = QGridLayout()
        pos_layout.setSpacing(5)
        
        self.obs_x_spin = QDoubleSpinBox()
        self.obs_x_spin.setRange(-10, 10)
        self.obs_x_spin.setValue(2.0)
        self.obs_x_spin.setSingleStep(0.5)
        
        self.obs_y_spin = QDoubleSpinBox()
        self.obs_y_spin.setRange(-10, 10)
        self.obs_y_spin.setValue(2.0)
        self.obs_y_spin.setSingleStep(0.5)
        
        self.obs_z_spin = QDoubleSpinBox()
        self.obs_z_spin.setRange(0, 5)
        self.obs_z_spin.setValue(0.0)
        self.obs_z_spin.setSingleStep(0.1)
        
        pos_layout.addWidget(QLabel("X:"), 0, 0)
        pos_layout.addWidget(self.obs_x_spin, 0, 1)
        pos_layout.addWidget(QLabel("Y:"), 1, 0)
        pos_layout.addWidget(self.obs_y_spin, 1, 1)
        pos_layout.addWidget(QLabel("Z:"), 2, 0)
        pos_layout.addWidget(self.obs_z_spin, 2, 1)
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # Obstacle size controls
        size_group = QGroupBox("Size")
        size_layout = QGridLayout()
        size_layout.setSpacing(5)
        
        self.obs_width_spin = QDoubleSpinBox()
        self.obs_width_spin.setRange(0.1, 5.0)
        self.obs_width_spin.setValue(1.0)
        self.obs_width_spin.setSingleStep(0.1)
        
        self.obs_height_spin = QDoubleSpinBox()
        self.obs_height_spin.setRange(0.1, 5.0)
        self.obs_height_spin.setValue(1.0)
        self.obs_height_spin.setSingleStep(0.1)
        
        self.obs_depth_spin = QDoubleSpinBox()
        self.obs_depth_spin.setRange(0.1, 5.0)
        self.obs_depth_spin.setValue(1.0)
        self.obs_depth_spin.setSingleStep(0.1)
        
        size_layout.addWidget(QLabel("W:"), 0, 0)
        size_layout.addWidget(self.obs_width_spin, 0, 1)
        size_layout.addWidget(QLabel("H:"), 1, 0)
        size_layout.addWidget(self.obs_height_spin, 1, 1)
        size_layout.addWidget(QLabel("D:"), 2, 0)
        size_layout.addWidget(self.obs_depth_spin, 2, 1)
        size_group.setLayout(size_layout)
        layout.addWidget(size_group)
        
        # Action buttons
        actions_group = QGroupBox("Actions")
        actions_layout = QVBoxLayout()
        actions_layout.setSpacing(5)
        
        add_obs_btn = QPushButton("➕ Add Obstacle")
        add_obs_btn.clicked.connect(self.add_obstacle_at_position)
        actions_layout.addWidget(add_obs_btn)
        
        preset_btn = QPushButton("📦 Add Preset Obstacles")
        preset_btn.clicked.connect(self.add_preset_obstacles)
        actions_layout.addWidget(preset_btn)
        
        clear_obs_btn = QPushButton("🗑️ Clear All Obstacles")
        clear_obs_btn.clicked.connect(self.clear_obstacles)
        actions_layout.addWidget(clear_obs_btn)
        
        actions_group.setLayout(actions_layout)
        layout.addWidget(actions_group)
        
        layout.addStretch()
        return tab
    
    def create_simulation_tab(self):
        """Create the simulation control tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)
        
        # Path planning controls
        planning_group = QGroupBox("Path Planning")
        planning_layout = QVBoxLayout()
        planning_layout.setSpacing(5)
        
        plan_btn = QPushButton("🗺️ Plan Path")
        plan_btn.clicked.connect(self.plan_path)
        planning_layout.addWidget(plan_btn)
        
        self.show_grid_check = QCheckBox("Show Occupancy Grid")
        self.show_grid_check.setChecked(False)
        self.show_grid_check.stateChanged.connect(self.update_visualization)
        planning_layout.addWidget(self.show_grid_check)
        
        planning_group.setLayout(planning_layout)
        layout.addWidget(planning_group)
        
        # Simulation controls
        sim_group = QGroupBox("Simulation Control")
        sim_layout = QVBoxLayout()
        sim_layout.setSpacing(5)
        
        self.start_btn = QPushButton("▶️ Start Simulation")
        self.start_btn.clicked.connect(self.toggle_simulation)
        sim_layout.addWidget(self.start_btn)
        
        speed_layout = QGridLayout()
        speed_layout.addWidget(QLabel("Speed:"), 0, 0)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(10)
        self.speed_spin.setSuffix(" Hz")
        speed_layout.addWidget(self.speed_spin, 0, 1)
        sim_layout.addLayout(speed_layout)
        
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        layout.addStretch()
        return tab
    
    def create_camera_tab(self):
        """Create the camera control tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)
        
        # Camera preset views
        preset_group = QGroupBox("Preset Views")
        preset_layout = QVBoxLayout()
        preset_layout.setSpacing(5)
        
        preset_layout.addWidget(QLabel("Quick Views:"))
        self.view_combo = QComboBox()
        self.view_combo.addItems(['Default', 'Top-Down', 'Side View (X)', 'Side View (Y)', 
                                   'Isometric', 'Follow Robot', 'Bird\'s Eye'])
        self.view_combo.currentTextChanged.connect(self.change_camera_view)
        preset_layout.addWidget(self.view_combo)
        
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)
        
        # Manual camera controls
        manual_group = QGroupBox("Manual Control")
        manual_layout = QGridLayout()
        manual_layout.setSpacing(5)
        
        manual_layout.addWidget(QLabel("Elevation:"), 0, 0)
        self.elev_spin = QSpinBox()
        self.elev_spin.setRange(-90, 90)
        self.elev_spin.setValue(30)
        self.elev_spin.setSuffix("°")
        self.elev_spin.valueChanged.connect(self.update_camera_manual)
        manual_layout.addWidget(self.elev_spin, 0, 1)
        
        manual_layout.addWidget(QLabel("Azimuth:"), 1, 0)
        self.azim_spin = QSpinBox()
        self.azim_spin.setRange(-180, 180)
        self.azim_spin.setValue(-60)
        self.azim_spin.setSuffix("°")
        self.azim_spin.valueChanged.connect(self.update_camera_manual)
        manual_layout.addWidget(self.azim_spin, 1, 1)
        
        manual_group.setLayout(manual_layout)
        layout.addWidget(manual_group)
        
        # Camera info
        info_label = QLabel(
            "<b>View Tips:</b><br>"
            "• Top-Down: Best for path planning<br>"
            "• Side Views: Check clearances<br>"
            "• Isometric: General viewing<br>"
            "• Follow Robot: Track movement"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("QLabel { background-color: #e8f4f8; padding: 8px; border-radius: 3px; font-size: 10px; }")
        layout.addWidget(info_label)
        
        layout.addStretch()
        return tab
    
    def on_robot_pos_changed(self):
        """Handle robot position change."""
        self.robot_pos[0] = self.robot_x_spin.value()
        self.robot_pos[1] = self.robot_y_spin.value()
        self.update_visualization()
    
    def on_robot_yaw_changed(self):
        """Handle robot yaw change."""
        self.robot_yaw = np.radians(self.robot_yaw_spin.value())
        self.update_visualization()
    
    def on_goal_pos_changed(self):
        """Handle goal position change."""
        self.goal_pos[0] = self.goal_x_spin.value()
        self.goal_pos[1] = self.goal_y_spin.value()
        self.update_visualization()
    
    def on_shape_changed(self, shape):
        """Handle obstacle shape change."""
        self.obstacle_shape = shape
    
    def add_obstacle_at_position(self):
        """Add an obstacle at the specified position."""
        position = [
            self.obs_x_spin.value(),
            self.obs_y_spin.value(),
            self.obs_z_spin.value()
        ]
        size = [
            self.obs_width_spin.value(),
            self.obs_height_spin.value(),
            self.obs_depth_spin.value()
        ]
        obstacle = Obstacle(position, size, self.obstacle_shape)
        self.obstacles.append(obstacle)
        self.update_visualization()
        self.status_label.setText(f"Added {self.obstacle_shape} at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}). Total: {len(self.obstacles)}")
    
    def add_preset_obstacles(self):
        """Add a preset configuration of obstacles."""
        presets = [
            Obstacle([2, 0, 0], [1, 1, 1], 'cube'),
            Obstacle([4, 2, 0], [1.5, 1.2, 1.5], 'cube'),
            Obstacle([3, -2, 0], [0.8, 1.5, 0.8], 'sphere'),
            Obstacle([1, 3, 0], [1, 1, 1], 'cube'),
            Obstacle([5, -1, 0], [1, 2, 1], 'cube'),
        ]
        self.obstacles.extend(presets)
        self.update_visualization()
        self.status_label.setText(f"Added preset obstacles. Total: {len(self.obstacles)}")
    
    def clear_obstacles(self):
        """Clear all obstacles."""
        self.obstacles.clear()
        self.path.clear()
        self.update_visualization()
        self.status_label.setText("Cleared all obstacles")
    
    def reset_robot(self):
        """Reset robot to starting position."""
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0
        self.robot_x_spin.setValue(0.0)
        self.robot_y_spin.setValue(0.0)
        self.robot_yaw_spin.setValue(0.0)
        self.path_index = 0
        self.update_visualization()
        self.status_label.setText("Robot reset to origin")
    
    def plan_path(self):
        """Plan a path from robot to goal."""
        # Generate point cloud from obstacles
        all_points = []
        for obs in self.obstacles:
            points = obs.generate_points(resolution=0.1)
            all_points.extend(points)
        
        if len(all_points) == 0:
            self.status_label.setText("No obstacles - direct path possible")
            # Create a direct path
            self.path = [
                (self.robot_pos[0], self.robot_pos[1]),
                (self.goal_pos[0], self.goal_pos[1])
            ]
            self.update_visualization()
            return
        
        all_points = np.array(all_points)
        
        # Update planner's grid
        self.planner.update_grid_from_pointcloud(all_points)
        
        # Plan path
        start = (self.robot_pos[0], self.robot_pos[1])
        goal = (self.goal_pos[0], self.goal_pos[1])
        self.path = self.planner.a_star(start, goal, self.robot_yaw)
        
        if self.path:
            self.status_label.setText(f"Path found! {len(self.path)} waypoints")
            self.path_index = 0
        else:
            self.status_label.setText("No path found! Try moving obstacles or goal.")
        
        self.update_visualization()
    
    def toggle_simulation(self):
        """Start or stop the simulation."""
        if not self.is_running:
            if not self.path:
                QMessageBox.warning(self, "No Path", "Please plan a path first!")
                return
            self.is_running = True
            self.start_btn.setText("Stop Simulation")
            self.path_index = 0
            self.timer.start(1000 // self.speed_spin.value())  # Convert Hz to ms
            self.status_label.setText("Simulation running...")
        else:
            self.is_running = False
            self.start_btn.setText("Start Simulation")
            self.timer.stop()
            self.status_label.setText("Simulation stopped")
    
    def update_simulation(self):
        """Update simulation step - move robot along path."""
        if not self.path or self.path_index >= len(self.path):
            self.toggle_simulation()  # Stop simulation
            self.status_label.setText("Goal reached!")
            return
        
        # Move robot to next waypoint
        next_pos = self.path[self.path_index]
        self.robot_pos[0] = next_pos[0]
        self.robot_pos[1] = next_pos[1]
        
        # Update UI
        self.robot_x_spin.blockSignals(True)
        self.robot_y_spin.blockSignals(True)
        self.robot_x_spin.setValue(self.robot_pos[0])
        self.robot_y_spin.setValue(self.robot_pos[1])
        self.robot_x_spin.blockSignals(False)
        self.robot_y_spin.blockSignals(False)
        
        # Calculate yaw to face next waypoint
        if self.path_index < len(self.path) - 1:
            next_next = self.path[self.path_index + 1]
            dx = next_next[0] - next_pos[0]
            dy = next_next[1] - next_pos[1]
            self.robot_yaw = np.arctan2(dy, dx)
            self.robot_yaw_spin.blockSignals(True)
            self.robot_yaw_spin.setValue(np.degrees(self.robot_yaw))
            self.robot_yaw_spin.blockSignals(False)
        
        self.path_index += 1
        self.update_visualization()
        self.status_label.setText(f"Waypoint {self.path_index}/{len(self.path)}")
    
    def change_camera_view(self, view_name):
        """Change camera to a preset view."""
        if view_name == 'Top-Down':
            self.elev_spin.setValue(90)
            self.azim_spin.setValue(-90)
        elif view_name == 'Side View (X)':
            self.elev_spin.setValue(0)
            self.azim_spin.setValue(0)
        elif view_name == 'Side View (Y)':
            self.elev_spin.setValue(0)
            self.azim_spin.setValue(-90)
        elif view_name == 'Isometric':
            self.elev_spin.setValue(35)
            self.azim_spin.setValue(-45)
        elif view_name == 'Follow Robot':
            # Position camera to follow robot
            self.elev_spin.setValue(20)
            self.azim_spin.setValue(int(np.degrees(self.robot_yaw) - 90))
        elif view_name == 'Bird\'s Eye':
            self.elev_spin.setValue(75)
            self.azim_spin.setValue(-60)
        elif view_name == 'Default':
            self.elev_spin.setValue(30)
            self.azim_spin.setValue(-60)
        
        self.update_visualization()
    
    def update_camera_manual(self):
        """Update camera based on manual spinbox values."""
        self.update_visualization()
    
    def update_visualization(self):
        """Update the 3D visualization."""
        self.ax.clear()
        
        # Set labels and limits
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([0, 5])
        self.ax.set_title('A* Obstacle Avoidance Simulator', fontsize=14, fontweight='bold')
        
        # Draw ground plane
        xx, yy = np.meshgrid(np.linspace(-10, 10, 10), np.linspace(-10, 10, 10))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')
        
        # Draw obstacles
        for i, obs in enumerate(self.obstacles):
            if obs.shape == 'cube':
                self.draw_cube(obs)
            elif obs.shape == 'sphere':
                self.draw_sphere(obs)
        
        # Draw robot
        self.draw_robot()
        
        # Draw goal
        self.ax.scatter([self.goal_pos[0]], [self.goal_pos[1]], [0.5],
                       c='green', marker='*', s=500, label='Goal', edgecolors='black', linewidths=2)
        
        # Draw path
        if self.path:
            path_array = np.array(self.path)
            self.ax.plot(path_array[:, 0], path_array[:, 1],
                        np.ones(len(path_array)) * 0.1,
                        'b-', linewidth=2, label='Planned Path')
            
            # Draw waypoints
            self.ax.scatter(path_array[:, 0], path_array[:, 1],
                           np.ones(len(path_array)) * 0.1,
                           c='blue', marker='o', s=50)
            
            # Highlight current waypoint in simulation
            if self.is_running and self.path_index < len(self.path):
                current = self.path[self.path_index]
                self.ax.scatter([current[0]], [current[1]], [0.1],
                               c='yellow', marker='o', s=200, edgecolors='black', linewidths=2)
        
        # Draw occupancy grid if enabled
        if self.show_grid_check.isChecked() and self.planner.occupancy_grid is not None:
            self.draw_occupancy_grid()
        
        self.ax.legend(loc='upper right')
        
        # Apply camera view settings
        self.ax.view_init(elev=self.elev_spin.value(), azim=self.azim_spin.value())
        
        self.canvas.draw()
    
    def draw_cube(self, obstacle):
        """Draw a cube obstacle."""
        vertices = obstacle.get_vertices()
        
        # Define the 6 faces of the cube
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
        ]
        
        # Create 3D polygon collection
        poly3d = Poly3DCollection(faces, alpha=0.7, facecolor='red', edgecolor='darkred', linewidths=1)
        self.ax.add_collection3d(poly3d)
    
    def draw_sphere(self, obstacle):
        """Draw a sphere obstacle."""
        x, y, z = obstacle.position
        radius = min(obstacle.size) / 2
        
        # Create sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        xs = radius * np.outer(np.cos(u), np.sin(v)) + x
        ys = radius * np.outer(np.sin(u), np.sin(v)) + y
        zs = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z
        
        self.ax.plot_surface(xs, ys, zs, color='red', alpha=0.7)
    
    def draw_robot(self):
        """Draw the robot with its footprint."""
        # Robot body (simple box)
        robot_height = 0.5
        corners_3d = []
        
        # Get robot footprint corners
        pose = (self.robot_pos[0], self.robot_pos[1], self.robot_yaw)
        corners_2d = self.planner.transform_corners(pose)
        
        # Create 3D vertices for robot
        for corner in corners_2d:
            corners_3d.append([corner[0], corner[1], 0])
        for corner in corners_2d:
            corners_3d.append([corner[0], corner[1], robot_height])
        
        # Draw robot faces
        faces = [
            [corners_3d[0], corners_3d[1], corners_3d[2], corners_3d[3]],  # Bottom
            [corners_3d[4], corners_3d[5], corners_3d[6], corners_3d[7]],  # Top
            [corners_3d[0], corners_3d[1], corners_3d[5], corners_3d[4]],  # Side 1
            [corners_3d[2], corners_3d[3], corners_3d[7], corners_3d[6]],  # Side 2
            [corners_3d[0], corners_3d[3], corners_3d[7], corners_3d[4]],  # Side 3
            [corners_3d[1], corners_3d[2], corners_3d[6], corners_3d[5]],  # Side 4
        ]
        
        poly3d = Poly3DCollection(faces, alpha=0.8, facecolor='cyan', edgecolor='blue', linewidths=2)
        self.ax.add_collection3d(poly3d)
        
        # Draw orientation arrow
        arrow_length = 0.5
        dx = arrow_length * np.cos(self.robot_yaw)
        dy = arrow_length * np.sin(self.robot_yaw)
        self.ax.quiver(self.robot_pos[0], self.robot_pos[1], robot_height/2,
                      dx, dy, 0, color='yellow', arrow_length_ratio=0.3, linewidths=3)
        
        # Add label
        self.ax.text(self.robot_pos[0], self.robot_pos[1], robot_height + 0.3,
                    'Robot', fontsize=10, fontweight='bold')
    
    def draw_occupancy_grid(self):
        """Draw the occupancy grid visualization."""
        grid = self.planner.occupancy_grid
        
        # Sample the grid for visualization (too many cells to show all)
        step = 10
        for gy in range(0, grid.shape[0], step):
            for gx in range(0, grid.shape[1], step):
                if grid[gy, gx] >= self.planner.obstacle_threshold:
                    x, y = self.planner.grid_to_world(gx, gy)
                    self.ax.scatter([x], [y], [0.05], c='orange', marker='s', s=20, alpha=0.3)


def main():
    """Main entry point for the simulator."""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    simulator = AStarSimulator()
    simulator.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

