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
- Making a service can be learnt [here](https://linuxhandbook.com/create-systemd-services/)

It works in a very sketchy way:
- I made a system service called `rover.service`
- The [file](services/rover.service) is for reference
    - The actual version is in `/etc/systemd/system` directory
- For some reason when I was launching all the commands in different tmux windows, the service would finish running and stop (which killed the ros node for some reason)
    - So I made joy.launch run in the end outside tmux session and now this works
- However, I cannot access the tmux session using the `tmux attach-session` commnd
- Also, the ssh command doesn't work in the service for some reason
- **If someone has time, figure out this stuff**

## Future idea: 
- Make a file that takes input from a button (either one of our bad controller or a button connected to an arduino)
- If the button is pressed, run the `manual_control` file
- Make a service that starts this file automatically on startup
- Result: A file is running in background waiting for input, once it gets input, it makes the rover drivable without using any laptop - this is for using the rover for showcasing, for fun, for ease of use
- When it doesn't get input, it doesn't do anything - this is when we are doing competition setup and we have laptops connected, we don't need to drive the rover around with a controller bluetooth connected to the rover. This can also help when we just want to debug
