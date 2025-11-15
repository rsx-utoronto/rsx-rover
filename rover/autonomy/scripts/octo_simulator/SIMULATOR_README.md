# A* Obstacle Avoidance Simulator

A standalone 3D simulator for testing and visualizing the A* pathfinding algorithm with obstacle avoidance. This simulator allows you to interactively place obstacles, set goals, and watch the robot navigate around them using the same A* algorithm from your ROS implementation.

![Simulator Preview](simulator_preview.png)

## Features

### 🎮 Interactive 3D Visualization
- Real-time 3D rendering of the environment
- Interactive camera controls (pan, zoom, rotate)
- Clear visualization of robot, obstacles, and planned paths

### 🚧 Obstacle Management
- **Two obstacle types**: Cubes and Spheres
- **Interactive placement**: Add obstacles with custom sizes
- **Preset configurations**: Quick test scenarios with pre-placed obstacles
- **Dynamic point cloud generation**: Each obstacle generates realistic point cloud data

### 🤖 Robot Simulation
- **Visual robot footprint**: See the robot's rectangular boundary
- **Orientation indicator**: Yellow arrow shows robot heading
- **Manual positioning**: Set robot position and orientation via controls
- **Automatic navigation**: Watch the robot follow the planned path

### 🗺️ Path Planning
- **A* algorithm**: Same implementation as your ROS node
- **Height-aware planning**: Considers terrain elevation differences
- **Collision checking**: Validates full robot footprint against obstacles
- **Real-time replanning**: Update paths as obstacles change

### 📊 Visualization Options
- **Occupancy grid overlay**: See the internal grid representation
- **Path waypoints**: Visualize individual waypoints along the path
- **Current waypoint highlighting**: Track progress during simulation

## Installation

### Prerequisites
- Python 3.7 or higher
- Windows 10/11 (tested on Windows)
- pip package manager

### Setup Steps

1. **Navigate to the scripts directory**:
   ```bash
   cd rover/autonomy/scripts
   ```

2. **Install required packages**:
   ```bash
   pip install -r simulator_requirements.txt
   ```

   Or install individually:
   ```bash
   pip install numpy matplotlib PyQt5
   ```

3. **Run the simulator**:
   ```bash
   python astar_simulator.py
   ```

## Usage Guide

### Getting Started

1. **Launch the simulator**: Run `python astar_simulator.py`
2. **Set up your scenario**:
   - Position the robot using the Robot Position controls
   - Set a goal using the Goal Position controls
   - Add obstacles (see below)
3. **Plan a path**: Click "Plan Path"
4. **Run simulation**: Click "Start Simulation" to watch the robot navigate

### Control Panel Overview

#### 🤖 Robot Position
- **X, Y**: Set the robot's starting position in meters
- **Yaw**: Set the robot's orientation in degrees

#### 🎯 Goal Position
- **X, Y**: Set the target destination in meters

#### 🚧 Obstacle Controls
- **Shape**: Choose between cube or sphere
- **Width/Height/Depth**: Set obstacle dimensions (0.1 - 5.0 meters)
- **Add Obstacle at Robot**: Place an obstacle near the robot's current position
- **Add Preset Obstacles**: Quickly add a test scenario with 5 obstacles
- **Clear All Obstacles**: Remove all obstacles from the scene

#### 🗺️ Path Planning
- **Plan Path**: Run the A* algorithm to find a path from robot to goal
- **Show Occupancy Grid**: Toggle visualization of the internal grid (sampled for performance)

#### ▶️ Simulation
- **Start/Stop Simulation**: Begin or pause the robot's movement along the path
- **Reset Robot Position**: Return robot to origin (0, 0, 0)
- **Speed**: Adjust simulation speed (1-100 Hz)

### Tips and Tricks

#### Creating Good Test Scenarios
1. Start with preset obstacles to understand the algorithm
2. Place obstacles between the robot and goal to test pathfinding
3. Try narrow passages to test the robot footprint collision checking
4. Add tall obstacles (high height values) to test elevation avoidance

#### Understanding the Visualization
- **Cyan box**: Robot with yellow orientation arrow
- **Red cubes/spheres**: Obstacles
- **Green star**: Goal position
- **Blue line with dots**: Planned path
- **Yellow dot** (during simulation): Current target waypoint
- **Orange squares** (when enabled): Occupied cells in the grid

#### Troubleshooting "No Path Found"
- Ensure the goal isn't inside an obstacle
- Check if obstacles completely block the path
- Try increasing the distance between obstacles
- Verify robot starting position is valid
- Make sure robot and goal are within the grid bounds (-10 to +10 meters)

## How It Works

### Algorithm Overview

The simulator implements the same A* pathfinding algorithm as your ROS node:

1. **Point Cloud Processing**:
   - Each obstacle generates a 3D point cloud (sampled points)
   - Points are filtered by height threshold (z_min: -0.25m, z_max: 3.0m)
   - Points are projected onto a 2D occupancy grid

2. **Occupancy Grid**:
   - Resolution: 0.1 meters per cell
   - Size: 200x200 cells (20m x 20m world space)
   - Centered at origin: -10m to +10m in X and Y

3. **A* Pathfinding**:
   - Uses 8-connected grid (can move diagonally)
   - Heuristic: Euclidean distance to goal
   - Cost function: Height difference + base movement cost (1.0)
   - Collision checking: Validates robot's full footprint at each pose

4. **Robot Footprint**:
   - Rectangular boundary: 0.6m x 0.6m (±0.3m from center)
   - Rotates with robot orientation
   - All four corners must be collision-free

### Key Parameters

```python
# Grid settings
grid_resolution = 0.1        # meters per cell
grid_size = (200, 200)      # cells

# Height filtering
z_min = -0.25               # meters
z_max = 3.0                 # meters

# Obstacle detection
obstacle_threshold = 100     # occupancy value

# Robot footprint
robot_corners = [
    [0.3, 0.3],   # front-right
    [0.3, -0.3],  # front-left
    [-0.3, -0.3], # back-left
    [-0.3, 0.3]   # back-right
]
```

### Differences from ROS Implementation

This simulator is functionally equivalent but adapted for standalone use:

| Feature | ROS Version | Simulator |
|---------|-------------|-----------|
| Input | ZED camera point cloud | Synthetic obstacle point clouds |
| Odometry | RTAB-Map odometry | Manual control or simulation |
| Output | ROS velocity commands | Visual animation |
| Visualization | RViz markers | Matplotlib 3D |
| Dependencies | ROS, octomap_msgs, etc. | Only NumPy, Matplotlib, PyQt5 |

## Extending the Simulator

### Adding New Obstacle Types

To add a new obstacle shape:

```python
# In Obstacle.generate_points()
elif self.shape == 'cylinder':
    # Your cylinder point generation code
    pass

# In AStarSimulator.update_visualization()
elif obs.shape == 'cylinder':
    self.draw_cylinder(obs)
```

### Modifying Cost Functions

To change how the algorithm weighs height differences:

```python
# In AStarPlanner.height_cost()
def height_cost(self, current, neighbor):
    # ... existing code ...
    height_diff = abs(...)
    
    # Modify this line to change cost calculation
    return height_diff * 2 + 1  # e.g., penalize height more
```

### Adjusting Robot Size

To change the robot footprint:

```python
# In AStarPlanner.__init__()
self.robot_corners = np.array([
    [0.5, 0.5],    # Larger robot
    [0.5, -0.5],
    [-0.5, -0.5],
    [-0.5, 0.5]
])
```

## Comparison with Original Algorithm

The simulator preserves the core logic from `astar_obstacle_avoidance_algorithim.py`:

### Maintained Features ✅
- Identical A* implementation
- Same occupancy grid structure
- Same robot footprint checking
- Same height cost calculation
- Same coordinate transformations

### Simplified for Testing ⚡
- No ROS message handling
- No real sensor integration
- Synthetic point cloud generation
- Manual control instead of autonomous navigation

This makes it perfect for:
- Testing algorithm changes without hardware
- Debugging pathfinding issues
- Demonstrating the algorithm to others
- Developing new features offline

## Performance Notes

- **Grid size**: Larger grids (>200x200) may slow down pathfinding
- **Point cloud density**: More points = slower grid updates but better accuracy
- **Visualization**: Showing the full occupancy grid is slow; we sample every 10th cell
- **Simulation speed**: Higher speeds (>50 Hz) may cause UI lag

## Troubleshooting

### Common Issues

**"Module not found" errors**:
```bash
pip install --upgrade numpy matplotlib PyQt5
```

**Blank window / No visualization**:
- Try different matplotlib backends
- Update graphics drivers
- Check if OpenGL is supported

**Simulator is slow**:
- Reduce number of obstacles
- Disable "Show Occupancy Grid"
- Lower simulation speed
- Close other applications

**Path planning fails**:
- Check console for error messages
- Verify obstacles aren't blocking all paths
- Ensure goal is reachable and valid
- Try simpler scenarios first

## Future Enhancements

Potential additions:
- [ ] Import/export obstacle configurations
- [ ] Multiple goal waypoints
- [ ] Dynamic obstacles that move
- [ ] Different robot shapes (circular, L-shaped)
- [ ] Terrain elevation maps
- [ ] Multiple path planning algorithms (Dijkstra, RRT)
- [ ] 3D pathfinding (not just 2D with height consideration)
- [ ] Performance metrics (path length, planning time, smoothness)

## License

This simulator is part of the RSX Rover project.

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the console output for error messages
3. Verify all dependencies are installed correctly
4. Test with preset obstacles first

---

**Happy Path Planning! 🤖🗺️**

