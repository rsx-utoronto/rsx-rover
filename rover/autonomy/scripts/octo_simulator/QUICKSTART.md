# Quick Start Guide - A* Simulator

Get up and running in 5 minutes!

## Installation

```bash
# Navigate to the scripts directory
cd rover/autonomy/scripts

# Install dependencies
pip install numpy matplotlib PyQt5

# Run the simulator
python astar_simulator.py
```

## First Test (30 seconds)

1. **Launch**: The simulator window opens with a 3D view
2. **Add obstacles**: Click "Add Preset Obstacles" button
3. **Set goal**: Already set to (5, 5) by default
4. **Plan path**: Click "Plan Path" button - you'll see a blue path appear
5. **Run**: Click "Start Simulation" - watch the robot navigate!

## Quick Tutorial (5 minutes)

### Add Your Own Obstacles

1. **Choose shape**: Select "cube" or "sphere" in dropdown
2. **Set size**: Adjust Width/Height/Depth (try 1.0m each)
3. **Position**: Move the robot to where you want the obstacle
4. **Add**: Click "Add Obstacle at Robot"
5. **Repeat**: Add several obstacles between robot and goal

### Move Things Around

**Robot**:
- X: 0 (starting position)
- Y: 0 
- Yaw: 0°

**Goal** (try different positions):
- X: 5, Y: 5 (diagonal)
- X: 8, Y: 0 (straight ahead)
- X: -3, Y: 4 (behind and to the side)

### Plan and Test

1. **Plan Path**: Click each time you move robot/goal/obstacles
2. **Start Simulation**: Watch the robot follow the path
3. **Speed**: Adjust slider (10 Hz is good for watching, 50 Hz for quick tests)

## Example Scenarios

### Scenario 1: Simple Obstacle
```
Robot: (0, 0)
Goal: (5, 0)
Obstacle: One cube at (2.5, 0) with size 1m x 1m x 1m
Result: Path goes around the obstacle
```

### Scenario 2: Maze
```
Robot: (0, 0)
Goal: (5, 5)
Obstacles: Add preset obstacles + add more to create narrow passages
Result: Path weaves through the gaps
```

### Scenario 3: No Path
```
Robot: (0, 0)
Goal: (5, 5)
Obstacles: Create a complete wall of cubes
Result: "No path found!" message - expected behavior!
```

## Controls Summary

| Button | What it does |
|--------|--------------|
| Plan Path | Run A* algorithm |
| Start Simulation | Animate robot movement |
| Reset Robot Position | Move robot back to (0,0) |
| Add Preset Obstacles | Quick test scenario |
| Clear All Obstacles | Remove everything |

## Common Questions

**Q: Why doesn't the robot move?**
A: Click "Plan Path" first, then "Start Simulation"

**Q: "No path found" message?**
A: Obstacles are blocking. Try "Clear All Obstacles" and start simpler.

**Q: Can't see the path?**
A: Make sure you clicked "Plan Path" and check the blue line appears

**Q: Robot jumps around weirdly?**
A: That's normal! It's following waypoints. Lower the simulation speed to watch more carefully.

**Q: What's the occupancy grid checkbox?**
A: Shows the internal grid used by A*. Orange squares = occupied cells.

## Tips

1. **Start simple**: Begin with 1-2 obstacles
2. **Use presets**: Great for learning how the algorithm works
3. **Try impossible**: Block the path completely to see failure mode
4. **Watch closely**: Lower simulation speed to see exact path following
5. **Read console**: Check the terminal for debug messages

## Next Steps

Once comfortable with basics:
- Read `SIMULATOR_README.md` for detailed documentation
- Experiment with different obstacle configurations
- Modify cost functions in the code
- Try different robot footprint sizes
- Test edge cases (robot starting in obstacle, goal in obstacle, etc.)

## Troubleshooting

**Simulator won't start?**
```bash
# Check Python version (need 3.7+)
python --version

# Reinstall dependencies
pip install --force-reinstall numpy matplotlib PyQt5
```

**Visual glitches?**
- Try clicking the "Home" button in the toolbar
- Rotate the 3D view with your mouse
- Restart the simulator

---

**You're ready to go! 🚀**

Start with preset obstacles and go from there!

