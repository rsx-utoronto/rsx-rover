# Arm Code

## Inverse Kinematics

### Terminal 1

```
cd ~/rover_ws # or whatever you named the ros workspace 
chmod +x src/rsx-rover/rover/arm/scripts/arm_master_control.py # makes file executable, only need to run the first time
rosocre
```

### Terminal 2

Actual Inverse Kinematics
```
cd ~/rover_ws
source devel/setup.bash
rosrun rover arm_master_control.py
```

### Terminal 3

The RViz simulation
```
cd ~/rover_ws
source devel/setup.bash
roslaunch rover arm_2023_rviz.laucnh
```