# microscopecam docs

## 1. Install packages

```bash
sudo apt install python3-opencv ros-noetic-cv-bridge v4l-utils
```

Packages installed:

1. `python3-opencv` opencv = open-source computer vision, used for accessing the camera feed

2. `ros-noetic-cv-bridge` converts opencv feed to ros message

3. `v4l-utils` is used for finding the correct camera ID


## 2. Connect camera

### 2.0

Plug in the camera.

### 2.1 VirtualBox USB pass through

**Skip this step if you are not using VirtualBox.**

In VirtualBox (while your VM is not running):\
Go to **Settings > USB**, select the **USB 3.0 (xHCI Controller)** box.

While your VM is running:\
Go to **Devices** at the top of the VM window, and select `AVEO_Technology_Corp._USB2.0_Camera`.


## 3. Run the publisher

Ensure that you have sourced into the devel environment, built the project with `catkin`, and `roscore` is running.


```bash
rosrun rover microscopecam.py
```

This will publish the camera feed to the `microscopecam` topic.

To see the published data, use `rviz`.
