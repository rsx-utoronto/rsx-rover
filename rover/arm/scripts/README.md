# Arm Code

## CAN Setup

To enable the CAN network, open a terminal and go the location of cloned repository

### Terminal 1
```
cd ~/rover-ws/src/rsx-rover/rover/arm/scripts/
./setup_can.bash
(Enter the password for your system)
```

### Terminal 2
To read CAN messages on the bus, type the following on terminal:
```
candump can0
```

### Terminal 3 (for debugging)
If the above command does not work, try sending a singular CAN packet on another terminal:
```
cansend can0 02050C80#FFFFFFFFFFFFFFFF
```
This is the message I usually prefer checking the candump with. If can connection is good, you should see the above message pop up in candump terminal

Once CAN network is enabled and you can see candump, go back to Terminal 1 again and type:
### Terminal 1
```
python3 arm_master_can.py
```

## Inverse Kinematics

### Running for the First Time

```
cd ~/rover_ws # or whatever you named the ros workspace 
chmod +x src/rsx-rover/rover/arm/scripts/arm_master_control.py # makes file executable, only need to run the first time
```

### For use with Real Arm
``` 
cd ~/rover_ws
source devel/setup.bash
roslaunch rover arm_2023_rviz.launch ik_on:=true
```

### Gazebo Prereqs

This is only needed to allow the griper to pickup objects in gazebo. I use a [plugin](https://github.com/JenniferBuehler/gazebo-pkgs) by Jennifer Buehler to allow the end-effector pick up stuff. Run the following commands to install the plugin (I lifted them from the github)

```
sudo apt install ros-noetic-gazebo-ros ros-noetic-eigen-conversions ros-noetic-object-recognition-msgs ros-noetic-roslint
cd ~/rover_ws/src # or whatever workspace you put the rover workspace in
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
cd ..
catkin_make
```

### For use in Gazebo
```
cd ~/rover_ws
source devel/setup.bash
roslaunch rover arm_2023_gazebo.launch ik_on:=true
```
