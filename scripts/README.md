# Manual Control Single File
- Use either `rosrun rover manual_control` or `source /home/rsx/rover_ws/src/rsx-rover/scripts/manual_control` to run this file
- To debug, you need to go into the tmux session
    - `tmux attach-session -t manual`
    - Ctrl+B N to move to different windows in tmux session
    - `tmux kill-session -t manual` to kill the session

- Trying to set it up so that `setup_can` doesn't require password
    - I added all the commands that require sudo in `setup_can` to superuser file
    - `sudo visudo`
    - This line is added by me: `rsx ALL = NOPASSWD: /sbin/ip link set down can0, /sbin/ip link set can0 type can bitrate 1000000, /sbin/ifconfig can0 txqueuelen 1000, /sbin/ip link set up can0, /home/rsx/rover_ws/src/rsx-rover/scripts/utils/gen/setup_can.sh, /home/rsx/rover_ws/src/rsx-rover/scripts/manual_control`
- Now the manual_control file can be used to run everything with one file

## Making it a system service

- `manual_control` doesn't work if this is a system service

It works in a very sketchy way:
- I made a system service called `rover.service`
- The [file](services/rover.service) is for reference
    - The actual version is in `/etc/systemd/system` directory
- For some reason when I was launching all the commands in different tmux windows, the service would finish running and stop (which killed the ros node for some reason)
    - So I made joy.launch run in the end outside tmux session and now this works
- However, I cannot access the tmux session using the `tmux attach-session` commnd
- Also, the ssh command doesn't work in the service for some reason
- **If someone has time, figure out this stuff**

## New Idea: 
- Make a file that takes input from a button (either one of our bad controller or a button connected to an arduino)
- If the button is pressed, run the `manual_control` file
- Make a service that starts this file automatically on startup
- Result: A file is running in background waiting for input, once it gets input, it makes the rover drivable without using any laptop - this is for using the rover for showcasing, for fun, for ease of use
- When it doesn't get input, it doesn't do anything - this is when we are doing competition setup and we have laptops connected, we don't need to drive the rover around with a controller bluetooth connected to the rover. This can also help when we just want to debug

- To take input from the controller, I made a new [launch file](../launch/joy_auto.launch)
- I have started writing [code](manual_switch.py) which takes input from controller
    - When you press the PS button, it runs the `manual_control` file which launches everything
    - Need to figure out how to kill this file - the break statement doesn't work
- I am making a new joy launch file which just launches for bluetooth software controller and I am hardcoding `js1` in it which means it always has to be the second controller connected
- This means the controller which gives the input to the rover for auto manual without computer is has to be always attached to the computer
- **Manual WORKS!!!!!**
- For some reason, the zed camera doesn't get launched (some problem with ssh) and I can't view the tmux screen for debugging (again for some reason idk)


## Final (for now)
### User Instructions
- Keep a controller wire connected to rover at all times 
    - or stop and disable the rover system service using `sudo systemctl stop rover` and `sudo systemctl disable rover`
- Press the PS button on the rover controller to launch manual stuff
- Connect the software controller (second controller) through bluetooth and enjoy

### Developer Instructions 
- The [manual_control](manual_control) file is for running drive and camera when we have laptops connected and we wanna do competition tasks
- The [bash file for service](manual_control_service) is to run when the `rover.service` is run
- This file runs the joy node and a [python script](manual_switch.py) that will wait in background until the PS button on the controler is pressed
- It will run [this script](manual_auto) to start ROS and manual control (without the camera)