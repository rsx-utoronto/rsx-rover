# GUI functions/descriptions

Some of the stuff we're currently working on!

## arm_gui_controller.py
- Edit the speed_multiplier variable to change how fast the arm moves
    - 6900 seems to work for manual mode
    - IK needs a much smaller value
- All the button functions work in reverse (ie. forwards moves backwards) in manual mode
- We can control some of the arm's individual joints with the individual buttons 

## qt6_gui.py

### in __init__:
- The topics in the camera_topic_name list need to be edited to work with actual robot

### in set_joint_display:
- Joint angles are not updated; connect to rest of code (WIP)

### in update_end_effector_coords:
- Uses instructions from ik_library function, but the values printed to the GUI are not accurate (WIP)

### in initUI:
- None of the Rx/Ry/Rz and open/close grip buttons work yet (WIP)
- Move to home/move to origin buttons not functional (WIP)
