# Arm Code

## Pre-Steps

- make sure the arm is connected to the CAN network
  - RSX uses a CAN USB like [this](https://www.amazon.ca/DSD-TECH-Adapter-Hardware-Canable/dp/B0BQ5G3KLR?pd_rd_w=MVljX&content-id=amzn1.sym.03d0a222-babc-4fe5-b7df-e90dd70bc007&pf_rd_p=03d0a222-babc-4fe5-b7df-e90dd70bc007&pf_rd_r=8DTA2VFZT7G3QNVWZZQ3&pd_rd_wg=6qoEh&pd_rd_r=cea5c5e8-ec16-4a88-9ab4-614c6502759c&pd_rd_i=B0BQ5G3KLR)
- make sure the arm has enough power
  - power supply capable of delivering at least 12V with a high current limit

## CAN Setup

To enable the CAN network, open a terminal and type the following (assuming you have the repository cloned)

### Terminal 1
```
cd ~/rover_ws/src/rsx-rover/scripts/utils/gen
./setup_can.sh
(Enter the password for your system)
```

### Terminal 2 (Optional Step)
To read CAN messages on the bus, type the following on terminal:
```
candump can0
```

#### Terminal 3 (for debugging)
If the above command does not work, try sending a singular CAN packet on another terminal:
```
cansend can0 02050C80#FFFFFFFFFFFFFFFF
```
This is the message I usually prefer checking the candump with. If can connection is good, you should see the above message pop up in candump terminal

Once CAN network is enabled and you can see candump, go back to Terminal 1 again and type:
### Terminal 1
```
roscore
```

### Terminal 2 (Assuming your joy node is setup. Refer to joy wiki for instructions on that)
```
export ROS_NAMESPACE=arm # makes this instance of the joy node in the arm workspace
rosrun joy joy_node
```

[Joy Wiki for Setup](https://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

#### Terminal 3 (Optional)
To check whether joy node is receiving inputs properly, run the following command
```
rostopic echo /arm/joy
```

### Terminal 3/4
Launch arm_controlller, manual and safety nodes
```
roslaunch rover arm_basics.launch
```

### Terminal 4/5
For gripper control, run gripper code
```
python3 ~/rover_ws/src/rsx-rover/rover/arm/scripts/gripper/gripper_controller.py
```

### Terminal 5/6
Run CAN_recv node (before CAN_send)
```
rosrun rover CAN_recv.py
```

Before proceeding further, make sure that all the angles that the arm thinks it are zero with:
```
rostopic echo /arm_curr_pos
```

### Terminal 6/7
Run CAN_send node
```
rosrun rover CAN_send.py
```
Enjoy! You will be able to directly use Manual mode

## Inverse Kinematics

### Running for the First Time

```
cd ~/rover_ws # or whatever you named the ros workspace 
chmod +x src/rsx-rover/rover/arm/scripts/arm_master_control.py # makes file executable, only need to run the first time
```

In addition you may want to edit the `goToPosition()` and `savePosition()` in arm_master_control.py to specify the whole path of arm_positions.json.

### For use with Real Arm
``` 
cd ~/rover_ws
catkin_make
source devel/setup.bash
roslaunch rover arm_rviz.launch ik_on:=true
```

### Gazebo Prereqs

This is only needed to allow the griper to pickup objects in gazebo. I used a [plugin](https://github.com/JenniferBuehler/gazebo-pkgs) by Jennifer Buehler to allow the end-effector pick up stuff. Run the following commands to install the plugin (I lifted them from the github)

```
sudo apt install ros-noetic-gazebo-ros ros-noetic-eigen-conversions ros-noetic-object-recognition-msgs ros-noetic-roslint
cd ~/rover_ws/src # or whatever workspace you put the rover workspace in
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
cd ..
catkin_make
```

### For use in Gazebo
The arm in gazebo comes with a 3D camera preattached to the fifth link of the arm. All you have to do to view the
output of the 3D camera is add a PointCloud2D to rivz and then select the topic.

```
cd ~/rover_ws
source devel/setup.bash
roslaunch rover arm_gazebo.launch ik_on:=true
```

## Using Inverse Kinematics

There are three ways to move the arm using the inverse kinematics node:
  1. Sending Coodinates for the end effector target
  2. Using a controller
  3. Using the keyboard

The following will outline how you can use each.

### Sending Coordiantes

In the future this functionality will go to path planning but for now it is in IK. You can give IK
an end effector position by publishing a Float32List to the ik_targets topic. The ik_targets topic
requires 7 element list in the following format [x, y, z, roll, pitch, yaw, gripperPos]. 

### Controller Control
Follow the [joy]([url](https://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)) package tutorial and configure you joystick to work with ROS.
```
# run the joy_node stuff
rosrun rover arm_controller.py # launch the arm_controllernode
roslauch rover arm_rviz.launch ik_on:=true # or use gazebo
```

Controls:
- Left joystick and both bumpers control [x,y, z]
- Right joystick upper bumpers control [roll, pitch, yaw]
- X and O controller gripper
- Press Triangle to change IK modes

### Keyboard Control
The keyboard controller mimics a controller inputs using your keyboard. I recommend opening the 
arm_keyboard_controller.py file to see what each keyboard button mimics.
```
rosrun rover arm_keyboard_controller.py
roslaunch rover arm_rviz.launch # or use gazebo
```
