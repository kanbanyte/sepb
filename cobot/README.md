
<!-- TOC ignore:true -->
# Cobot Package
<!-- TOC ignore:true -->
## Overview
The cobot package contains ROS2 code that allows for the ZED camera and UR5e cobot to interact and perform the pick and place task.
There are two ROS2 packages within:
[`pick_place_package`](src/pick_place_package/README.md) which contains all of the nodes, the launch file, config files, and additional scripts, and
[`pick_place_interfaces`](src/pick_place_interfaces/README.md) which contains ROS2 interfaces for communication between nodes.

**Table of Contents**
<!-- TOC -->

* [Dependencies](#dependencies)
* [Installation](#installation)
* [Usage](#usage)
* [Common Issues](#common-issues)

<!-- /TOC -->

## Dependencies
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble#universal-robots-ros2-driver)

## Installation
<!-- Maybe add part here about cloning workspace from repo correctly so bash cobot.sh functions work correctly. -->

Navigate the the directory where the `cobot` workspace is located to build the workspace.

Source the `cobot.sh` file for access to relevant functions:
```bash
source `cobot.sh`
```

Source the ROS2 `setup.bash` file:
```bash
init_ros2
```

Build the package with this simple command:
```bash
rebuild_cobot
```

This is equivalent to this:
```bash
rm -r $COBOT_BUILD_DIR
rm -r $COBOT_INSTALL_DIR
rm -r $COBOT_LOG_DIR
cd $COBOT_PATH
colcon build
source install/local_setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

## Usage
<!-- TOC ignore:true -->
### Directory Structure
The following list contains a concise explanation of the directory structure:
* [pick_place_interfaces](src/pick_place_interfaces/README.md): contains any/all `.msg`, `.srv`, and `.action` files.
* [pick_place_package](src/pick_place_package/README.md): contains launch files, config files, nodes, and additional scripts.

The `cobot.sh` file provides many functions to do numerous things with regards to the robot.

The [launch file](src/pick_place_package/launch/pick_place_launch.py) starts three nodes:
* [`camera_node`](src/pick_place_package/pick_place_package/camera_node.py)
* [`cobot_node`](src/pick_place_package/pick_place_package/cobot_node.py)
* [`gripper_node`](src/pick_place_package/pick_place_package/gripper_node.py)

The pick and place task is started from [`main_node`](src/pick_place_package/pick_place_package/main_node.py), which is run in a separate terminal.

<!-- TOC ignore:true -->
### Starting the Robot Driver
Before performing the pick and place task, start the robot driver and connect to the robot:
```bash
connect_cobot
```

You should see six messages similar to the following at the bottom of the terminal.
If there are six messages, there shouldn't be a problem controlling the robot.
```bash
[INFO] [spawner-11]: process has finished cleanly [pid 31924]
[INFO] [spawner-9]: process has finished cleanly [pid 31881]
[INFO] [spawner-8]: process has finished cleanly [pid 31920]
[INFO] [spawner-10]: process has finished cleanly [pid 31877]
[INFO] [spawner-6]: process has finished cleanly [pid 31879]
[INFO] [spawner-7]: process has finished cleanly [pid 31856]
```

<!-- TOC ignore:true -->
### Performing the Pick and Place Task
If you have already built the package, run the following command to run the launch file:
```bash
launch_cobot
```

If you have modified the code and need to rebuild, you can run the following command to rebuild the package and immediately start the cobot:
```bash
start_cobot
```

Wait until no more messages are outputted in the terminal, then in another terminal run the following command to run the main node:
```bash
run_main
```

## Common Issues
<!-- TOC ignore:true -->
### Trouble Starting Robot Driver
**Symptoms:**\
* The terminal doesn't have six `finished with pid [xxxx]` messages at the end.
* Continuous output of `configure gpio tf-prefix`.

**Solution:**\
Ensure that the `ROS.urp` program is loaded on the controller.
You may need to keep restarting the driver until you see the six messages as stated above.

<!-- TOC ignore:true -->
### Gripper Not Working During Task
**Symptoms:**\
The gripper doesn't open or close during the task even if the terminal in which the launch file was run shows that it is calling the gripper service.

**Solution:**\
Sometimes the `play` service isn't called due to an error in the driver so the `gripper_test.URP` program isn't called.
Kill the terminals running the launch file and main node.
Restart the robot driver then run the launch file and main node again.
