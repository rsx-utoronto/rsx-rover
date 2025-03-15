# Manual Control Rover

The drive sender node is used to send velocity commands to the `/drive` topic

## Currently:

- The left joystick controls the linear and angular velocity
    - the y vector controls the linear velocity
    - the x vector controls the angular velocity
- R1 to increase the gear
- L1 to decrease the gear
- PS button to stop the rover

## Goal:

**GTA Mode:**
- R2 to accelerate, L2 to decelarate 
	- If both, move at that constant speed
    - If none, stop the rover with max decelearation
- R1 to increase gear, L1 to decrease gear
- Left joystick to control steering
	- the y vector is not used
	- the x vector is used to: 
		- control steering (more x, more steering)
		- control lnear speed (more x, less lin speed)
        - More x, more angular acceleration
- Right Dpad to turn completely on spot (linear speed = 0) clockwise
- Left Dpad to turn completely on spot (linear speed = 0) counter-clockwise

**Easy Mode:**
- Left joystick for all actions
- R1 to increase gear
- L1 to decrease gear


- By default, starts with Easy Mode
- To switch modes, press R1, L1 and Triangle together
- Reimplement the network status thing so that rover stops moving as soon as we get disconnected from the network

### Notes:

- Removed the *5 from max speed (which was for the gear ratio) because I observed that rover reached max speed at 2.5
- Changed the robot radius to 1 (it was 0.8 before) which is accurate for current iteration of the rover
- Set maximum acceleration to 0.075 - can be changed based on testing
- There's a problem: the code only does stuff when the controller is getting input (changing things on the controller) however, when the controller state is constant (like there is button pressed or R2 is pressed completely), joy topic doesn't get any input and the algorithm doesn't run
    - To fix this, the algorithm needs to be outside the joy callback function
    - Done
- New bug: When the controller is connected, before R2 and L2 have not been pressed, the R2 and L2 value on the axes shows 0 instead of 1 (which is the idle state). This causes the rover to move as soon as the gear is changed even when L2 and R2 are not pressd.
    - I noticed that buttons[7] becomes 1 when R2 is pressed and buttons[6] becomes 1 when L2 is pressed
    - **Solved!** Added this in the if statements for positive and negative throttle
- Another bug: Software control is **SHIT**: When I press R2, it only goes to 0 (instead of -1) and L2 goes to -0.9. The arm controler is fine, R2 and L2 go to -1.
    - I am not hardcoding anything right now but it could be done
- Bug: When I run autonomoy, it gives jerky movement showing that 0 velocity is regularly published
    - This is because I set up the manual code to keep on publishing 0s even when there is no input on the controller
    - Fix: Don't publish anything once the robot has reached 0 velocity until a new command from controller is given



