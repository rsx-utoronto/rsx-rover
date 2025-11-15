# Usage Examples and Scenarios

Detailed examples for testing various aspects of the A* obstacle avoidance algorithm.

## Example 1: Basic Obstacle Avoidance

**Objective:** Test simple obstacle avoidance with a single cube.

### Setup
```
Robot Position:  X=0, Y=0, Yaw=0°
Goal Position:   X=5, Y=0
Obstacle:        Cube at (2.5, 0), size 1m × 1m × 1m
```

### Steps
1. Set robot to (0, 0)
2. Set goal to (5, 0)
3. Add a 1m cube obstacle
4. Use robot position controls to place obstacle at (2.5, 0)
5. Click "Plan Path"

### Expected Result
- Path should go around the obstacle (above or below in Y direction)
- Approximately 6-8 waypoints
- Path should not pass through the obstacle

### Screenshot
```
         Goal (5,0)
           *
          /
    [■]  /
    |   /
   (0,0) Robot
```

---

## Example 2: Diagonal Navigation

**Objective:** Test pathfinding through narrow diagonal passage.

### Setup
```
Robot:     (0, 0)
Goal:      (8, 8)
Obstacles: 
  - Cube at (3, 2), 1.5m × 1m × 1.5m
  - Cube at (5, 5), 1.5m × 1m × 1.5m
  - Cube at (7, 3), 1.5m × 1m × 1.5m
```

### Steps
1. Reset robot to origin
2. Set goal to (8, 8)
3. Add three cubes at positions listed above
4. Click "Plan Path"
5. Enable "Show Occupancy Grid" to see grid representation

### Expected Result
- Path weaves between obstacles
- Takes advantage of diagonal movements
- ~10-15 waypoints
- Occupancy grid shows three red zones

---

## Example 3: Impossible Path

**Objective:** Test algorithm behavior when no path exists.

### Setup
```
Robot: (0, 0)
Goal:  (5, 0)
Wall:  5 cubes in a line blocking the goal
```

### Steps
1. Robot at (0, 0)
2. Goal at (5, 0)
3. Add cubes at:
   - (3, -3)
   - (3, -1.5)
   - (3, 0)
   - (3, 1.5)
   - (3, 3)
4. Click "Plan Path"

### Expected Result
- Status shows "No path found!"
- Console prints "A* failed to find a path"
- No blue path line appears
- This is correct behavior - goal is unreachable

### Fix
- Remove one cube to create a gap
- Re-plan: path should now go through the gap

---

## Example 4: Height-Based Costs

**Objective:** Demonstrate height difference penalties.

### Setup
```
Robot: (0, 0)
Goal:  (6, 0)

Path A (flat):
  - Cube at (2, 0.5), height = 0.5m
  - Cube at (4, 0.5), height = 0.5m

Path B (steep):
  - Empty at (2, -0.5)
  - Empty at (4, -0.5)
```

### Steps
1. Add two short cubes (height = 0.5m) above the direct path
2. Leave lower path clear
3. Plan path

### Expected Result
- Path prefers the lower route (less height change)
- Even though direct distance is shorter through cubes
- This shows the height cost function working

### Variation
- Change cube heights to 2.0m
- Path should strongly avoid these now
- Try heights: 0.5m, 1.0m, 1.5m, 2.0m to see cost differences

---

## Example 5: Robot Footprint Testing

**Objective:** Verify that the robot's rectangular footprint is properly checked.

### Setup
```
Robot: (0, 0, 0°)
Narrow passage: Two cubes with 0.8m gap
```

### Steps
1. Create narrow passage:
   - Cube at (3, 0.5), size 1m × 1m × 1m
   - Cube at (3, -0.5), size 1m × 1m × 1m
2. Gap between them: ~0.8m
3. Robot footprint: 0.6m × 0.6m
4. Set goal to (6, 0)
5. Plan path

### Expected Result
- Path should go through the gap (0.8m > 0.6m robot width)
- Watch robot animation to verify it fits

### Variation - Narrower Gap
1. Move cubes closer:
   - Cube at (3, 0.4)
   - Cube at (3, -0.4)
2. Gap now: ~0.5m
3. Re-plan

### Expected Result
- No path found! (0.5m < 0.6m robot width)
- Robot footprint prevents passage

---

## Example 6: Dynamic Replanning

**Objective:** Simulate dynamic obstacle discovery.

### Scenario: "New obstacle appears"

### Steps
1. Start with preset obstacles
2. Plan path (should succeed)
3. Start simulation
4. PAUSE simulation mid-way (Stop button)
5. Add new obstacle directly in robot's path
6. Re-plan path
7. Resume simulation

### Expected Result
- New path avoids the new obstacle
- Demonstrates replanning capability
- In real robot: this happens automatically as new point clouds arrive

---

## Example 7: Corner Cases

### 7A: Robot Starts in Obstacle
```
1. Add obstacle at (0, 0)
2. Try to plan path
3. Expected: "Start position is invalid!" message
```

### 7B: Goal in Obstacle
```
1. Add obstacle at goal position
2. Try to plan path
3. Expected: "Goal position is invalid!" message
```

### 7C: Robot Very Close to Obstacle
```
1. Place obstacle 0.35m away (just outside footprint)
2. Should still plan path
3. Place obstacle 0.25m away (inside footprint)
4. Should fail
```

### 7D: Spherical Obstacles
```
1. Change shape to "sphere"
2. Add several spheres
3. Path should navigate around curved surfaces
4. Compare with equivalent cubes
```

---

## Example 8: Performance Testing

**Objective:** Test algorithm with many obstacles.

### Setup
```
Robot: (0, 0)
Goal:  (9, 9)
Obstacles: 20-30 cubes randomly placed
```

### Steps
1. Reset simulator
2. Add preset obstacles (5)
3. Manually add 15-25 more obstacles throughout map
4. Plan path
5. Note planning time in console

### Expected Performance
- Planning time: 50-500ms depending on complexity
- More obstacles = longer planning time
- Very dense obstacles may result in no path

### Optimization Test
```
1. Plan path with 5 obstacles   → Time: ~50ms
2. Plan path with 10 obstacles  → Time: ~100ms
3. Plan path with 20 obstacles  → Time: ~200ms
4. Plan path with 50 obstacles  → Time: ~500ms+
```

---

## Example 9: Simulation Speed Comparison

**Objective:** Understand simulation speed parameter.

### Test Different Speeds
```
Speed = 1 Hz:   Very slow, 1 waypoint per second
Speed = 10 Hz:  Moderate, good for observation
Speed = 50 Hz:  Fast, quick testing
Speed = 100 Hz: Very fast, may cause UI lag
```

### Steps
1. Plan any path with ~10 waypoints
2. Set speed to 1 Hz
3. Start simulation - watch carefully
4. Repeat with 10 Hz, 50 Hz, 100 Hz

### Observations
- Lower speeds: easier to see exact robot movement
- Higher speeds: faster testing iteration
- Recommended: 10 Hz for demos, 50 Hz for testing

---

## Example 10: Real-World Scenario

**Objective:** Simulate a realistic rover scenario.

### Scenario: "Navigate to sample location"

### Environment Setup
```
Start:  (0, 0) - Rover landing site
Goal:   (8, 6) - Sample location
Obstacles:
  - Large rock at (3, 2), 2m × 2m × 2m
  - Boulder field: 5 small rocks around (5, 4)
  - Crater edge: line of cubes at Y=-1
```

### Steps
1. Clear all obstacles
2. Add large rock (2m cube) at (3, 2)
3. Add 5 small rocks (0.5m cubes) scattered around (5, 4)
4. Add "crater edge": 5 cubes in a line at Y=-1
5. Set goal to (8, 6)
6. Plan path
7. Run simulation

### Expected Result
- Path avoids large rock by going around
- Weaves through boulder field
- Stays away from crater edge
- ~15-20 waypoints
- Total path length: ~12-15 meters

### Variations
- **Night scenario**: Add more obstacles (reduced visibility)
- **Rough terrain**: Use taller obstacles (height > 1.5m)
- **Narrow canyon**: Create walls on both sides

---

## Example 11: Algorithm Tuning

**Objective:** Understand how parameters affect behavior.

### Parameter: Grid Resolution

**Current: 0.1m**
```python
# In code: self.grid_resolution = 0.1
```

**Effect:**
- Finer resolution (0.05m): More accurate, slower planning
- Coarser resolution (0.2m): Faster, but less precise

### Parameter: Obstacle Threshold

**Current: 100**
```python
# In code: self.obstacle_threshold = 100
```

**Effect:**
- Higher threshold: More conservative (wider berth around obstacles)
- Lower threshold: More aggressive (closer to obstacles)

### Parameter: Height Min/Max

**Current: -0.25m to 3.0m**
```python
self.z_min = -0.25
self.z_max = 3.0
```

**Effect:**
- Narrower range: Ignores very low/high points
- Wider range: Considers more terrain features

---

## Example 12: Debugging Failed Paths

**Problem:** "No path found" when you expect one.

### Debugging Steps

1. **Enable Occupancy Grid**
   - Check "Show Occupancy Grid"
   - Orange squares show occupied cells
   - Verify obstacles are where you expect

2. **Check Robot Footprint**
   - Click "Plan Path"
   - Watch green bounding box
   - Ensure robot fits through gaps

3. **Verify Positions**
   - Print start/goal coordinates
   - Check they're within -10 to +10 range
   - Ensure they're not in obstacles

4. **Simplify Scenario**
   - Clear all obstacles
   - Add one at a time
   - Find which obstacle breaks the path

5. **Check Console Output**
   - Look for error messages
   - Check coordinate conversions
   - Watch for "invalid pose" messages

---

## Tips for Effective Testing

### 1. Start Simple
- Begin with 1-2 obstacles
- Use preset configurations first
- Gradually increase complexity

### 2. Use Systematic Testing
- Change one parameter at a time
- Document what works and what doesn't
- Create test cases you can repeat

### 3. Visualize Everything
- Enable occupancy grid when debugging
- Watch the simulation at slow speed first
- Use the 3D view controls to inspect from different angles

### 4. Compare with ROS
- Test same scenarios in simulator and on robot
- Verify paths are similar
- Note any differences and investigate

### 5. Edge Cases Matter
- Test boundary conditions
- Try impossible scenarios
- Verify error handling works

---

## Advanced Exercises

### Exercise 1: Maze Navigation
Create a maze using cubes and find the path through it.

### Exercise 2: Multiple Goals
Manually create a multi-waypoint mission by planning to several goals in sequence.

### Exercise 3: Moving Start
Start simulation, pause, move robot, re-plan, continue.

### Exercise 4: Minimum Gap
Find the minimum gap size the robot can pass through (should be ~0.61m).

### Exercise 5: Longest Path
Create a scenario where the shortest geometric path is blocked, forcing a much longer route.

---

## Conclusion

These examples demonstrate:
- ✅ Basic obstacle avoidance
- ✅ Complex scenarios
- ✅ Edge case handling  
- ✅ Performance characteristics
- ✅ Real-world applicability

Use these as templates for your own testing scenarios!

