# RSX Zenoh Setup

## 1. Install Zenoh
In new terminal, run the following command to install Zenoh:
```bash
sudo apt update
sudo apt install ros-<distro>-rmw-zenoh-cpp
```
Replace `<distro>` with your ROS distribution (e.g., `humble`).

## 2. Set Environment Variables
To change the ROS2 RMW (middleware) implementation to Zenoh, you need to set the environment variables in the env_var.sh file. You can add these lines to your `~/.bashrc` file to make them permanent (highly recommended). Alternatively, you can source the env_var.sh file in each terminal before running ROS2 commands.

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=5
export ZENOH_SESSION_CONFIG_URI=~/rover_ws/src/rsx-rover/zenoh_doc/zenoh_config.json5
```

**Make sure to change the path to `zenoh_config.json5` if you have it located elsewhere.**

### 2.1 Running Nodes Locally Without Connecting to Jetson

To do this, change the endpoint ip in the `zenoh_config.json5` file to `127.0.0.1` (localhost). and either re-source your .bashrc file if you have added the environment variables there:

```bash
source ~/.bashrc
```
 or export the ZENOH_SESSION_CONFIG_URI manually:
```bash
export ZENOH_SESSION_CONFIG_URI=~/rover_ws/src/rsx-rover/zenoh_doc/zenoh_config.json5
``` 
**Replace the path with the actual location of your `zenoh_config.json5` file.** This will allow you to run ROS2 nodes on your local machine without connecting to the Jetson.

Zenoh operates like ros1 (the roscore part), so you would need to run a zenoh router (similar to roscore) before running any ROS2 nodes. You can run the zenoh router with the following command:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```
or 
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
```
The `&` allows the router to run in the background, so you can continue using the terminal for other commands.

## 3. Router Setup

For Zenoh to work, you need to have a router running. The router can be run on any machine that has network access to the other machines running ROS2 nodes, But in our case, **we will run the router on the Jetson.** 

For the router to work, you need to have an additional configuration file that specifies the router's behavior. This file is called `zenoh_router_config.json5` and should be located in the same directory as the `zenoh_config.json5` file (see `zenoh_router_config.json5`).

Another environment variable needs to be set to point to the router configuration file:

```bash
export ZENOH_ROUTER_CONFIG_URI=~/rover_ws/src/rsx-rover/zenoh_doc/zenoh_router_config.json5
```
**Make sure to change the path to `zenoh_router_config.json5` if you have it located elsewhere.**

To run the router on the Jetson, simply SSH into the Jetson and run the following command:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**This is needed for any ros2 nodes in the network to run**