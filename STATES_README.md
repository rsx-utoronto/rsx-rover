# State Machine Test For ArUco

## Step 1.
run state_publisher.py in rover/autonomy/scripts and make sure the self.state_msg.rover_mode in it is set to "AUTONOMY" for the rover to go into Init state as soon as it recieves a message.

## Step 2.
run state_machine.py in rover/autonomy/scripts.

The rover should switch between the following states in order:

1) Idle
2) Init

It will ask for GPS coordinates. Already pre-set to [0,0,0], just type `Y` and then enter.

3) ArucoTraverse
4) GPSCheckPoint
5) ArucoScan

At this point it should be continuously going back on itself and searching for a tag. When there is a tag found by the Zed Camera, there should be a topic published from ar_detection_node.py in over/autonomy/scripts. After a tag is found the states should continue on.

6) Transition

Because of the logic inside state_machine.py the "MISSION OVER" will not run if `num_gps_goals` is not set to 1. If it is, the state machine will exit thinking the competition tasks are over. If not, it will ask for more GPS waypoints. At this point the states will loop starting from the Init state.

## Possible Bugs
There are some values manually set to True or False in state_machine.py to get the state to the desired test case. All of them should have a comment above them saying `#remove`. If you want to change the conditions to get to another case, you can check the state change conditions under their respective `exec()` functions.

Hopefully none of them will result in a bug.