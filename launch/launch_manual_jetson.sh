#!/bin/bash

# NOTE: This script must be launched on the jetson through ssh 

# Make sure to update this list when new plugins are added
# See instructions at bottom

# Setup environment 
cd ~/catkin_ws
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.250:11311
export ROS_IP=192.168.0.250

# Do not start roscore unless specified
core=0
record=0
topic_list=()

# Parse optional args
while [[ $# -gt 0 ]]
do
	key="$1"
	case $key in
		-h|--help)
			echo -e "launch_drive bash script V.2.1 (2022 Aug 5, Catherine Glossop, based on work of Eddie Tian)\n"
			echo -e "Usage: launch_drive [arguments]\n"
			echo -e "Known Issue: Only way to stop nodes is with 'top' then 'kill {PID}'. A workaround is to close the terminal window, then run 'launch_drive' again."
			echo -e "Arguments"
			echo -e "\t-h|--help               Show this help screen"
			echo -e "\t-c|--core               Run roscore (it's best to run roscore separately because you can't"
			echo -e "\t                        stop jobs started in scripts with \$jobs). Default is run."
			echo -e "Each node is launched and a tmux panel is created for echoing topics and checking status of topics"
			exit 0
		;;
		-c|--core)
			core=1
			;;
		*)
			echo "Invalid argument. See launch_drive --help for help."
			exit 1
	esac
	shift
done


# Start roscore, if --core option set
if [ $core == 1 ]
then
	echo "Starting roscore..."
	roscore &
fi

# Start up motors 
echo "Starting up falcons..."
roslaunch rover falcons.launch &

# Start drive_sender
echo "Starting drive_sender..."
rosrun rover drive_sender_falcons &

# Start sensors
echo "Launching sensors..."
roslaunch rover sensors.launch &

# wheel odom node launch 

# state est node launch 

# tf tree launch 

tmux \
  new-session -s "drive" -n "control" \; \
  send-keys "rostopic echo drive" C-m \; \
  split-window -h -p 50 \; \
  send-keys "rostopic echo joy" C-m \; \
  new-window -n "motors" \; \
  send-keys "rostopic echo /front_left/status" C-m \; \
  split-window -v -p 33 \; \
  send-keys "rostopic echo /mid_left/status" C-m \; \
  split-window -v -p 33 \; \
  send-keys "rostopic echo /back_left/status" C-m \; \
  select-layout even-vertical \; \
  select-pane -t 0 \; \
  split-window -h\; \
  send-keys "rostopic echo /front_right/status" C-m \; \
  select-pane -t 2 \; \
  split-window -h \; \
  send-keys "rostopic echo /mid_right/status" C-m \; \
  select-pane -t 4 \; \
  split-window -h \; \
  send-keys "rostopic echo /back_right/status" C-m \; \
  new-window -n "sensors" \; \
  send-keys "rostopic echo /fix" C-m \; \
  split-window -h \; \
  send-keys "rostopic echo /imu" C-m \;
  select-pane -t 0 \; \ 
  split-window -v \; \
  send-keys "rostopic hz /zed/zed_node/rgb/image_rect_color" C-m \; \ 
  select-pane -t 2 \; \
  split-window -v \; \ 
  send-keys "rostopic hz /os_cloud_node/points" C-m \;