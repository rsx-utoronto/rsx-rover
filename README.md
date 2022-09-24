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

#### Step 0: Before cloning
Make sure you have the following installed:
1. [Git](https://gitforwindows.org/)
2. [Docker Desktop](https://www.docker.com/products/docker-desktop/)
3. [Xming](https://sourceforge.net/projects/xming/)(windows) or [XQuartz](https://www.xquartz.org/)(macOS, not tested)

#### Step 1: Clone repo
Use git bash to clone the repo 
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

### Step 3: Set up your display variables
Start a bash terminal or powershell 
```
ipconfig
```
Find the ip address of your host 
```
export DISPLAY=<your ip address>:0.0
```

### Step 4: Run the container 
```
./rsx_docker_run.sh
```
This will interactively run the docker container. You can either use VScode's remote - container extension to connect or continue in the terminal 

### Step 5: Test
```
roscore&
rviz
```
The rviz gui should open on your host device 



