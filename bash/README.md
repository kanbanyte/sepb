
<!-- TOC ignore:true -->
# Bash Files
<!-- TOC ignore:true -->
## Overview
The bash directory contains files defining any bash functions that help with the usage and development of the pick and place task.

**Table of Contents**
<!-- TOC -->

* [ai.sh](#aish)
* [cobot.sh](#cobotsh)
* [path.sh](#pathsh)
* [setup.sh](#setupsh)
* [variables.sh](#variablessh)

<!-- /TOC -->

## ai.sh
[ai.sh](/bash/ai.sh) contains functions concerning the AI portion of the project.
Three functions are defined:
* `rebuild_ai()`: Deletes the AI build directory to avoid problems when installing the AI package.
* `install_ai()`: Installs the AI package using `pip install .`.
* `start_ai()`: First calls `rebuild_ai()` then `install_ai()`; deletes the build directory then installs again.
Use this function after making changes to the AI package.

## cobot.sh
[cobot.sh](/bash/cobot.sh) contains many functions concerning the cobot and ROS2.
Many of these functions call services to save time when testing or starting the cobot.
#!/bin/bash

* `ping_cobot()`: Ping cobot twice.
* `init_ros2()`: Initialise ROS2.
* `launch_driver()`: Launch cobot driver.
* `connect_cobot()`: Establish cobot connection.
* `load_ros_file()`: If not done manually, load the program ROS.urp which allows remote control.
This can be done manually, or via command line.
If done via command line makes the driver crash, so restart the robot driver.
* `power_on_cobot()`: Power on the robot motors.
* `release_brake()`: Release break.
* `play_cobot()`: Plays the current program.
**IMPORTANT:** Stop and start the program if it is already running.
This was causing the robot not moving for no reason.
Make sure you stop and start the program if you are restarting the driver.
* `stop_cobot()`: Pauses the current program.	
* `switch_controllers()`: Switches controller to `scaled_joint_trajectory_controller`.
* `launch_rviz()`: Launch MoveIt with Rviz.
**IMPORTANT:** If the Rviz robot position is not like the real robot, something has gone wrong.
So, restart the driver, stop, play, switch controller, MoveIt.
* `rebuild_cobot()`: Rebuild cobot package.
* `launch_cobot()`: Launch cobot package.
* `start_cobot()`: Rebuild cobot package then launch.
* `run_camera()`: Run the camera node.
* `run_cobot()`: Run the cobot node.
* `run_gripper()`: Run the gripper node.
* `run_main()`: Run the main node.
* `service_call_case()`: Case service call.
* `service_call_chip()`: Chip service call.	
* `service_call_tray()`: Tray service call.	
* `send_goal_action()`: Send a goal to perform pick place action.
* `load_gripper_file()`: Load gripper file for tests.
* `close_fully()`: Completely close gripper.
* `open_fully()`: Completely open gripper.
* `open_partly()`: Open gripper for tray.
* `open_grip()`: Open gripper for battery.
* `pinch_grip()`: Pinch gripper for chip.

## path.sh
[path.sh](/bash/path.sh) contains the path to where the repository is stored and sources the other bash files in the bash directory.

## setup.sh
[setup.sh](/bash/setup.sh) finds the location of [cobot.sh](/bash/cobot.sh), traverses up the directory structure to the repository directory,
and overrides the `REPO_PATH` variable in [path.sh](/bash/path.sh).
It then sources path.sh in ~/.bashrc to access the bash functions in any terminal.

## variables.sh
[variables.sh](/bash/variables.sh) contains path variables for use in the other bash files.

