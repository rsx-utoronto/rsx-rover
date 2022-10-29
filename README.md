# Robotics for Space Exploration (RSX)

## What is RSX?

Robotics for Space Exploration (RSX) is the University of Toronto Faculty of Applied Science and Engineering’s affiliated Mars Rover design team. 
RSX designs, builds, and competes with Mars Rovers in international competitions, such as the University Rover Challenge, European Rover Challenge, and Canadian International Rover Challenge. 
RSX was formed in 2013 by passionate roboticists, space enthusiasts, and undergraduate students. RSX challenges students to be innovative, striving to push boundaries in extraterrestrial exploration. We encourage students to learn about, experiment with, and develop proficiency in engineering technology, and promote growth through hands-on experience. 

For us, the sky is not the limit – it’s just in the way.

For more information, visit our [website](https://rsx.skule.ca/). 

## Repository Details 

We primarily develop is C++ and Python using ROS noetic and ROS2 Humble. 

## Getting Started 

Note: It is recommended to set up [ssh keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) 

### On Ubuntu Focal w/ ROS noetic 
```
cd ~
mkdir -p rover_ws/src 
git clone git@github.com:rsx-utoronto/rsx-rover.git
cd ..
catkin_make 
```
### On Windows or Mac 
Note: On Mac Docker cannot access the gpu, so anything that requires it will not work (i.e. Gazebo)

### Step 0: Before cloning
Make sure you have the following installed:
1. [Git](https://gitforwindows.org/)
2. [Docker Desktop](https://www.docker.com/products/docker-desktop/)
3. [Xming](https://sourceforge.net/projects/xming/) (windows) or [XQuartz](https://www.xquartz.org/) (macOS)

### Step 1: Clone repo
Use git bash to clone the repo (to your local machine)
```
git clone git@github.com:rsx-utoronto/rsx-rover.git
```

### Step 2: Build docker image from Dockerfile 
Start bash 
```
cd rsx-rover/docker
./docker_build.sh
```
You should get a docker image called rsx_dev_rsx

Note: this will take about 15 minutes as it needs to install ROS onto the nvidia image 

### Step 3: Start X server

Windows: 
Start up Xming and accept all the default configurations and save your configuration to the Xming folder. Make sure the display variable is 0.
Mac: 
Start up XQuartz and navigate to Preferences -> Security. Make sure to check 'Allow connections from network clients' before restarting XQuartz.

### Step 4: Run the container 
Windows:
Use the rsx_docker_run.sh script to create a running container from the docker image (built in step 2). 

> Refer to the [instructions for using rsx_docker_run.sh](docker/running_image_directions.md)

This will interactively run the docker container. You can either use VScode's remote - container extension to connect or continue in the terminal 

Mac:
Start bash
```
ip=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $ip
docker run -it --name rsx_dev -e DISPLAY=$ip:0 -v 'path to rsx-rover':/home/rsx/rover_ws/src/rsx-rover rsx_dev_rsx
```
Make sure to change 'path to rsx-rover' to where you cloned the repo on your device (ex. /Users/username/rsx-rover), this mounts it to the docker container. 

### Step 5: Test
```
roscore&
rviz
```
The rviz gui should open on your host device. 



