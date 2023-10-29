<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System for a Pick and Place Task
<!--
	Author: @finnmcgearey
	Editor(s): @dau501
	Year: 2023
-->

`Technical Documentations`

<!-- TOC ignore:true -->
## Industry Project 24
|Name|Position|Email|
|:-|:-|:-|
|@Slothman1|Team Leader/Client Liaison|id@swin.student.edu.au|
|@dau501|Development Manager/Planning Manager|id@swin.student.edu.au|
|@finnmcgearey|Support Manager/Developer|id@swin.student.edu.au|
|@vkach|Quality Manager/Developer|id@swin.student.edu.au|
|@NickMcK14|Support Manager/Developer|id@swin.student.edu.au|
|@Huy-GV|Quality Manager/Developer|id@swin.student.edu.au|

<div class="page"/><!-- page break -->

<!-- TOC ignore:true -->
# Table of Contents
<!-- TOC -->

* [Introduction](#introduction)
* [Bash Files](#bash-files)
	* [ai.sh](#aish)
	* [cobot.sh](#cobotsh)
	* [path.sh](#pathsh)
	* [setup.sh](#setupsh)
	* [variables.sh](#variablessh)
* [Cobot Package](#cobot-package)
	* [Dependencies](#dependencies)
	* [Installation](#installation)
	* [Usage](#usage)
	* [Common Issues](#common-issues)
* [Pick and Place Interfaces](#pick-and-place-interfaces)
	* [PickPlaceAction.action](#pickplaceactionaction)
	* [PickPlaceService.srv](#pickplaceservicesrv)
* [Pick and Place Package](#pick-and-place-package)
	* [Config](#config)
	* [Launch](#launch)
	* [Nodes](#nodes)
	* [pick_place_package](#pick_place_package)
	* [Related Terminal Commands](#related-terminal-commands)
* [Node APIs](#node-apis)
	* [camera_server.py](#camera_serverpy)
	* [cobot_methods.py](#cobot_methodspy)
	* [cobot_movement.py](#cobot_movementpy)
	* [gripper_server.py](#gripper_serverpy)
	* [main_action_client.py](#main_action_clientpy)
	* [read_methods.py](#read_methodspy)
* [Pick and Place Packages](#pick-and-place-packages)
	* [camera_node.py](#camera_nodepy)
	* [cobot_node.py](#cobot_nodepy)
	* [gripper_node.py](#gripper_nodepy)
	* [main_node.py](#main_nodepy)
* [AI Package](#ai-package)
	* [Dependencies](#dependencies-1)
	* [Installation](#installation-1)
	* [Usage](#usage-11)
	* [Common Issues](#common-issues-1)
* [Camera APIs](#camera-apis)
	* [camera_capture.py](#camera_capturepy)
	* [Capturing Images via Terminal](#capturing-images-via-terminal)
* [Data Processing APIs](#data-processing-apis)
	* [case_position.py](#case_positionpy)
	* [image_processing.py](#image_processingpy)
	* [tray_position.py](#tray_positionpy)
	* [chip_position.py](#chip_positionpy)
* [Model APIs](#model-apis)
	* [detected_object.py](#detected_objectpy)
	* [object_detection_model.py](#object_detection_modelpy)
* [Sample object detection programs](#sample-object-detection-programs)
	* [basic_sample.py](#basic_samplepy)
	* [camera_sample.py](#camera_samplepy)
	* [sample_config.yaml](#sample_configyaml)
* [Training Data Processing Scripts](#training-data-processing-scripts)
	* [dataset_balancer.py](#dataset_balancerpy)
	* [copy_interval.py](#copy_intervalpy)
	* [define_crop.py](#define_croppy)
	* [random_crop.py](#random_croppy)
* [Training](#training-1)
	* [object_detection_training.ipynb](#object_detection_trainingipynb)
	* [plot_ultralytics_results.py](#plot_ultralytics_resultspy)
	* [trained](#trained)
* [Util APIs](#util-apis)
	* [file_dialog.py](#file_dialogpy)
	* [file_reader.py](#file_readerpy)

<!-- /TOC -->

<div class="page"/><!-- page break -->

# Introduction
This technical document contains all information regarding the project: *Robot Vision System for a Pick and Place Task*.
It acts as both a user manual and installation manual, providing information about each file and directory to help with setting up the packages and using them.
Each important file and any functions have been explained to ensure the best possible usage.

<div class="page"/><!-- page break -->

# Bash Files
<!-- TOC ignore:true -->
## Overview
The bash directory contains files defining any bash functions that help with the usage and development of the pick and place task.

## ai.sh
[ai.sh](/bash/ai.sh) contains functions concerning the AI portion of the project.
Three functions are defined:
* `rebuild_ai()`: Deletes the AI build directory to avoid problems when installing the AI package.
* `install_ai()`: Installs the AI package using `pip install .`.
* `start_ai()`: First calls `rebuild_ai()` then `install_ai()` which deletes the build directory then installs again.
Use this function after making changes to the AI package.

## cobot.sh
[cobot.sh](/bash/cobot.sh) contains many functions concerning the cobot and ROS2.
Many of these functions call services to save time when testing or starting the cobot.
* `ping_cobot()`: Ping cobot twice.
* `init_ros2()`: Initialise ROS2.
* `launch_driver()`: Launch cobot driver.
* `connect_cobot()`: Establish cobot connection.
* `load_ros_file()`: If not done manually, load the program ROS.urp which allows remote control.
This can be done manually, or via command line.
If done via command line makes the driver crash, so restart the robot driver.
* `power_on_cobot()`: Power on the robot motors.
* `release_brake()`: Release brake.
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
[setup.sh](/bash/setup.sh) finds the location of [cobot.sh](/bash/cobot.sh),
traverses up the directory structure to the repository directory, and overrides the `REPO_PATH` variable in [path.sh](/bash/path.sh).
It then sources `path.sh` in `~/.bashrc` to access the bash functions in any terminal.

## variables.sh
[variables.sh](/bash/variables.sh) contains path variables for use in the other bash files.

<div class="page"/><!-- page break -->

# Cobot Package
<!-- TOC ignore:true -->
## Overview
The cobot package contains ROS2 code that allows for the ZED camera and UR5e cobot to interact and perform the pick and place task.
There are two ROS2 subpackages within:
[`pick_place_package`](/cobot/src/pick_place_package/README.md) which contains all of the nodes, the launch file, config files, and additional scripts, and
[`pick_place_interfaces`](/cobot/src/pick_place_interfaces/README.md) which contains ROS2 interfaces for communication between nodes.

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
* [pick_place_interfaces](/cobot/src/pick_place_interfaces/README.md): contains any/all `.msg`, `.srv`, and `.action` files.
* [pick_place_package](/cobot/src/pick_place_package/README.md): contains launch files, config files, nodes, and additional scripts.

The `cobot.sh` file provides many functions to do numerous things with regards to the robot.

The [launch file](/cobot/src/pick_place_package/launch/pick_place_launch.py) starts three nodes:
* [`camera_node`](/cobot/src/pick_place_package/pick_place_package/camera_node.py)
* [`cobot_node`](/cobot/src/pick_place_package/pick_place_package/cobot_node.py)
* [`gripper_node`](/cobot/src/pick_place_package/pick_place_package/gripper_node.py)

The pick and place task is started from [`main_node`](/cobot/src/pick_place_package/pick_place_package/main_node.py),
which is an action client running in a separate terminal.

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

<div class="page"/><!-- page break -->

# Pick and Place Interfaces
<!-- TOC ignore:true -->
## Overview
The pick and place interfaces package contains custom interfaces for use within [`pick_place_package`](/cobot/src/pick_place_package/README.md);
that allows each node to communicate with each other appropriately.
The package contains two directories:
* `action` which contains the action file for communicating between [`main_node`](/cobot/src/pick_place_package/pick_place_package/main_node.py) and
[`cobot_node`](/cobot/src/pick_place_package/pick_place_package/cobot_node.py).
* `srv` which contains the service file for communicating between [`cobot_node`](/cobot/src/pick_place_package/pick_place_package/cobot_node.py) and
[`camera_node`](/cobot/src/pick_place_package/pick_place_package/camera_node.py).

## PickPlaceAction.action
<!-- TOC ignore:true -->
### Purpose
`PickPlaceAction.action` is an action file that allows `main_node` to initiate the pick and place task.
It sends a boolean as a goal that is used to start the task, it returns the name of the current movement as feedback, and
returns a boolean for whether the task was successful or not.

<!-- TOC ignore:true -->
### Usage
It is used in both `main_node` and `cobot_node`.
`main_node` sends a goal to the `cobot_node` which then returns the name of the current trajectory as feedback before returning the task's success.

## PickPlaceService.srv
<!-- TOC ignore:true -->
### Purpose
`PickPlaceService.srv` is a service file for use with the `chip`, `case`, and
`tray` services as defined in [`camera_server.py`](/cobot/src/pick_place_package/nodes/camera_server.py).
It uses a boolean to request the camera node to detect the relevant objects, and
returns an `int64` as a response corresponding to a chip position, case position, or tray movement.

<!-- TOC ignore:true -->
### Usage
It is used in both `camera_node` and `cobot_node`.
`cobot_node` requests a case, chip, or tray position from `camera_node` which then returns the signal as a response.

<div class="page"/><!-- page break -->

# Pick and Place Package
<!-- TOC ignore:true -->
## Overview
The pick and place package contains all config files, launch files, nodes, and
additional files related to ROS2 to allow the cobot to move and communicate with the ZED camera.

## Config
<!-- TOC ignore:true -->
### Purpose
The config directory contains any config files that will be used within the package.
The only config file used currently is `cobot_config.yaml`,
which contains all joint positions and gripper pin outputs to move the cobot and control the gripper respectively.

<!-- TOC ignore:true -->
### Usage
Place any config files such as `.yaml` files in this directory.
The path to the config directory is already included in [`setup.py`](/cobot/src/pick_place_package/setup.py).

## Launch
<!-- TOC ignore:true -->
### Purpose
The launch directory contains any launch files.
`pick_place_launch.py` launches the gripper, camera, and cobot nodes in that order and also passes the config file as a parameter into the relevant nodes.

<!-- TOC ignore:true -->
### Usage
Place any launch files in this directory.
Ensure they are named in this format: `[file_name]_launch.py` or `[file_name].launch.py`.
The path to the launch directory is already included in [`setup.py`](/cobot/src/pick_place_package/setup.py).

## Nodes
<!-- TOC ignore:true -->
### Purpose
The nodes directory contains the classes for each custom node as well as additional files used by these nodes.
The nodes are spun in files contained within the [`pick_place_package`](/cobot/src/pick_place_package/pick_place_package/README.md) directory.

<!-- TOC ignore:true -->
### Usage
Place any node class definitions within this directory.

## pick_place_package
<!-- TOC ignore:true -->
### Purpose
The pick_place_package directory contains the `main` methods that spin the nodes defined in the nodes directory.

<!-- TOC ignore:true -->
### Usage
Whenever a new node is defined, place the `main` method that spins the node in this directory.
Make sure you include the entry point within `setup.py` as shown below:
```py
entry_points={
	'console_scripts': [
		'cobot_node = pick_place_package.cobot_node:main',
		'camera_node = pick_place_package.camera_node:main',
		'gripper_node = pick_place_package.gripper_node:main',
		'main_node = pick_place_package.main_node:main'
	],
},
```

## Related Terminal Commands
The following demonstrates terminal commands and their outputs.\
These services assists in testing the Camera Server without having to start the Cobot Action Server.

<!-- TOC ignore:true -->
### Case Service Call
```bash
ros2 service call /case pick_place_interfaces/srv/PickPlaceService "{detect: True}"
```

> Output.

```txt
waiting for service to become available...
requester: making request: pick_place_interfaces.srv.PickPlaceService_Request(detect=True)

response:
pick_place_interfaces.srv.PickPlaceService_Response(signal=11)
```

<!-- TOC ignore:true -->
### Chip Service Call
```bash
ros2 service call /chip pick_place_interfaces/srv/PickPlaceService "{detect: True}"
```

> Output.

```txt
requester: making request: pick_place_interfaces.srv.PickPlaceService_Request(detect=True)

response:
pick_place_interfaces.srv.PickPlaceService_Response(signal=7)
```

<!-- TOC ignore:true -->
### Tray Service Call
```bash
ros2 service call /tray pick_place_interfaces/srv/PickPlaceService "{detect: True}"
```

> Output.

```txt
waiting for service to become available...
requester: making request: pick_place_interfaces.srv.PickPlaceService_Request(detect=True)

response:
pick_place_interfaces.srv.PickPlaceService_Response(signal=5)
```

<!-- TOC ignore:true -->
### List All Services
```bash
ros2 service list
```

> Output.

```txt
/cobot_node/describe_parameters
/cobot_node/get_parameter_types
/cobot_node/get_parameters
/cobot_node/list_parameters
/cobot_node/set_parameters
/cobot_node/set_parameters_atomically
/camera_node/describe_parameters
/camera_node/get_parameter_types
/camera_node/get_parameters
/camera_node/list_parameters
/camera_node/set_parameters
/camera_node/set_parameters_atomically
/case
/chip
/controller_manager/configure_controller
/controller_manager/describe_parameters
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/list_hardware_components
/controller_manager/list_hardware_interfaces
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/set_hardware_component_state
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/controller_manager/switch_controller
/controller_manager/unload_controller
/controller_stopper/describe_parameters
/controller_stopper/get_parameter_types
/controller_stopper/get_parameters
/controller_stopper/list_parameters
/controller_stopper/set_parameters
/controller_stopper/set_parameters_atomically
/dashboard_client/add_to_log
/dashboard_client/brake_release
/dashboard_client/clear_operational_mode
/dashboard_client/close_popup
/dashboard_client/close_safety_popup
/dashboard_client/connect
/dashboard_client/describe_parameters
/dashboard_client/get_loaded_program
/dashboard_client/get_parameter_types
/dashboard_client/get_parameters
/dashboard_client/get_robot_mode
/dashboard_client/get_safety_mode
/dashboard_client/list_parameters
/dashboard_client/load_installation
/dashboard_client/load_program
/dashboard_client/pause
/dashboard_client/play
/dashboard_client/popup
/dashboard_client/power_off
/dashboard_client/power_on
/dashboard_client/program_running
/dashboard_client/program_saved
/dashboard_client/program_state
/dashboard_client/quit
/dashboard_client/raw_request
/dashboard_client/restart_safety
/dashboard_client/set_parameters
/dashboard_client/set_parameters_atomically
/dashboard_client/shutdown
/dashboard_client/stop
/dashboard_client/unlock_protective_stop
/force_torque_sensor_broadcaster/describe_parameters
/force_torque_sensor_broadcaster/get_parameter_types
/force_torque_sensor_broadcaster/get_parameters
/force_torque_sensor_broadcaster/list_parameters
/force_torque_sensor_broadcaster/set_parameters
/force_torque_sensor_broadcaster/set_parameters_atomically
/forward_position_controller/describe_parameters
/forward_position_controller/get_parameter_types
/forward_position_controller/get_parameters
/forward_position_controller/list_parameters
/forward_position_controller/set_parameters
/forward_position_controller/set_parameters_atomically
/io_and_status_controller/describe_parameters
/io_and_status_controller/get_parameter_types
/io_and_status_controller/get_parameters
/io_and_status_controller/hand_back_control
/io_and_status_controller/list_parameters
/io_and_status_controller/resend_robot_program
/io_and_status_controller/set_io
/io_and_status_controller/set_parameters
/io_and_status_controller/set_parameters_atomically
/io_and_status_controller/set_payload
/io_and_status_controller/set_speed_slider
/io_and_status_controller/zero_ftsensor
/joint_state_broadcaster/describe_parameters
/joint_state_broadcaster/get_parameter_types
/joint_state_broadcaster/get_parameters
/joint_state_broadcaster/list_parameters
/joint_state_broadcaster/set_parameters
/joint_state_broadcaster/set_parameters_atomically
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/scaled_joint_trajectory_controller/describe_parameters
/scaled_joint_trajectory_controller/get_parameter_types
/scaled_joint_trajectory_controller/get_parameters
/scaled_joint_trajectory_controller/list_parameters
/scaled_joint_trajectory_controller/query_state
/scaled_joint_trajectory_controller/set_parameters
/scaled_joint_trajectory_controller/set_parameters_atomically
/speed_scaling_state_broadcaster/describe_parameters
/speed_scaling_state_broadcaster/get_parameter_types
/speed_scaling_state_broadcaster/get_parameters
/speed_scaling_state_broadcaster/list_parameters
/speed_scaling_state_broadcaster/set_parameters
/speed_scaling_state_broadcaster/set_parameters_atomically
/tray
/ur_tool_comm/describe_parameters
/ur_tool_comm/get_parameter_types
/ur_tool_comm/get_parameters
/ur_tool_comm/list_parameters
/ur_tool_comm/set_parameters
/ur_tool_comm/set_parameters_atomically
```

<!-- TOC ignore:true -->
### List All Nodes
```bash
ros2 node list
```

> Output.

```txt
/cobot_node
/camera_node
/controller_manager
/controller_stopper
/dashboard_client
/force_torque_sensor_broadcaster
/forward_position_controller
/io_and_status_controller
/joint_state_broadcaster
/robot_state_publisher
/scaled_joint_trajectory_controller
/speed_scaling_state_broadcaster
/ur_tool_comm
```

<!-- TOC ignore:true -->
### List All Actions
```bash
ros2 action list
```

> Output.

```txt
/perform_pick_place
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

<!-- TOC ignore:true -->
### Send a Goal to Perform Pick Place Action
```bash
ros2 action send_goal /perform_pick_place pick_place_interfaces/action/PickPlaceAction "{perform_pick_place: True}"
```

> Output.

```txt
Waiting for an action server to become available...
Sending goal:
	perform_pick_place: true

Goal accepted with ID: aede4df792b24a0a9c195e60c6dff389

Result:
	task_successful: false

Goal finished with status: SUCCEEDED
```

<div class="page"/><!-- page break -->

# Node APIs
## camera_server.py
This class is a node acting as a server that provides positions of items upon request.
Object positions or tray movements can be requested via three different services:
tray service, case service, and chip service, all of which share the same service interface.
The returned integer signal can be interpreted differently based on the called service:
* Chip service: returned signal is a number from 1 to 48 if a chip is detected in these positions or -1 if none is detected.
* Case service: returned signal is a number from 1 to 17 if a case is detected in these positions or -1 if none is detected.
* Tray service: returned signal is the value from the enum `CobotMovement` from the AI package.

The services use the models and camera settings specified in a configuration YAML file.
Change the paths in the constructor of the class ([CameraServer](/cobot/src/pick_place_package/nodes/camera_server.py)) if you like to use your own.
Refer to the samples provided by the AI package for more information about camera and model configurations.

## cobot_methods.py
This module contains functions that populate the trajectories of the cobot.
There are 2 main types of trajectories:
* Tray load: loads the chip, case, and PCB shell on to the tray.
* Tray move: move the trays, including moving trays and 2 to assembly and vice versa.

## cobot_movement.py
This class is a node acting as the Cobot Movement Action server which triggers a pick-and-place task upon request.
The server depends on values returned by the Camera Server to determine which item to pick-up or move.

It is worth noting that the server only performs a single action per request, and an action can only be the following:
* Move tray: this action moves trays 1 and 2 to assembly and vice versa.
* Load tray: this action loads all items onto an empty tray (either tray 1 or tray 2).

The action selection is based on the suggested movement returned from the Camera Server.
To keep the cobot performing the pick-and-place task in an infinite loop, the client must send action requests in an infinite loop.
The class also supports a dummy action callback (`execute_test`) which does not involve the cobot or the camera server, primarily used to test action clients.

## gripper_server.py
This class is a server for the gripper service that allows the cobot node to control the gripper.

It works by using four main functions:
1. `load_file(file_name)`: Loads the specified file on the PLC.
2. `play_program()`: Plays the currently loaded file on the PLC.
3. `switch_pin_io(request)`: Sends a pin number as a request to the `io_and_status_controller/set_io` service.
4. `load_and_play_program(file_name)`: Loads a file then plays it.

On the PLC, a program called 'gripper_test.urp' has been created that runs a loop to check which digital output pins are active.
When the `gripper_service` service is called, it runs the `gripper_callback` function which sets the pin number and activates it.
Once the pin has been activated, the gripper will open or close as defined in 'gripper_test.urp'.

Once the pin number and state has been sent, the 'ROS.urp' program is loaded played, giving control back to the PC to move the cobot.

## main_action_client.py
This class acts as an action client node which sends pick-and-place request to the Cobot Movement Action Server and waits until that request is complete.
The result and status of the request are printed to the terminal once the request is completed.
Feedback detailing the cobot movements during the process are also provided.

## read_methods.py
This file contains functions that read information from `cobot_config.yaml`.
The two functions defined are:
* `read_positions_from_parameters(action_server, goal_names)`: Reads the joint positions for each goal name.
* `read_gripper_outputs_from_parameters(action_server, output_names)`: Reads the gripper outputs for each goal name.

These functions allow the cobot node to use the goals and gripper information defined in the config file to move to specific positions and
move the gripper certain distances.

<div class="page"/><!-- page break -->

# Pick and Place Packages
## camera_node.py
<!-- TOC ignore:true -->
### Purpose
Camera node is a ROS2 Node that supplies the position of objects when requested.\
This node is built upon the class [CameraServer](/cobot/src/pick_place_package/nodes/camera_server.py) and
must be running before the Cobot Action Server receives a request.

<!-- TOC ignore:true -->
### Usage
The camera node is rebuilt and started along with the Cobot Action Server if the system is started with the command `start_cobot`.

## cobot_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [CobotActionServer](/cobot/src/pick_place_package/nodes/cobot_movement.py) class,
to act as an action server retrieving requests to perform a pick-and-place task.

<!-- TOC ignore:true -->
### Usage
This node is rebuilt and started with all its dependencies if the system is started with the command `start_cobot`.

## gripper_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [GripperServer](/cobot/src/pick_place_package/nodes/gripper_server.py) class, to act as a server that opens or closes the gripper upon a request.\
This node is used by the Cobot Action Server to grab items before moving them.

<!-- TOC ignore:true -->
### Usage
The gripper node is rebuilt and started along with the Cobot Action Server if the system is started with the command `start_cobot`.

## main_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [MainActionClient](/cobot/src/pick_place_package/nodes/main_action_client.py) class, to act as a client to the Cobot Movement Action Server.\
It sends requests to the action server in an infinite loop to keep the cobot running constantly.

<!-- TOC ignore:true -->
### Usage
The client node is rebuilt and started via the command `run_main`.\
If a request fails, the user will be asked whether to continue the loop in the terminal.

<div class="page"/><!-- page break -->

# AI Package
<!-- TOC ignore:true -->
## Overview
The AI Package contains scripts and modules developed to interact with the ZED camera, run inference using detection models as well as process training data.
Image labelling is done in a web application known as Roboflow whereas training is done in a GPU-supported Google Collab session.

## Dependencies
<!-- TOC ignore:true -->
### Hardware Requirements
* A computer running on an NVIDIA GPU with CUDA support.

<!-- TOC ignore:true -->
### Python Packages
* To ensure that the machine has packages to run every script in this directory, run the following command:
	```bash
	# replace path/to/requirements.txt
	python3 -m pip install -r path/to/requirements.txt
	```

<!-- TOC ignore:true -->
### ZED2 SDK
* Install the ZED2 SDK from [the official site](https://www.stereolabs.com/developers/release/).
	* Install the CUDA version suggested by the SDK and do not skip it.
	The installation continues if you choose to ignore CUDA but the code will not work.
* Install the Python API by running the `get_python_api.py` script:
	* On Windows, the script is in `C:\Program Files (x86)\ZED SDK\`
	* On Linux, the script is in `/usr/local/zed/`
	* See the [official documentation](https://www.stereolabs.com/docs/app-development/python/install/#installing-the-python-api) for more information.

## Installation
* Note: use `python` if you are running in Windows and `python3` if you are running in Linux.
This document exclusively uses the Linux version.
* To ensure you have the latest pip, run the command:
	```bash
	python3 -m pip install --upgrade pip
	```
* To install the package, navigate to the project containing `pyproject.toml`, remove the any existing `build` directory and run the below command.
This command should be run every time the package code is modified:
	```bash
	python3 -m pip install .
	```

## Usage
<!-- TOC ignore:true -->
### Directory Structure
The below list provides an overview of the directory structure.
See the `README.md` files within individual subdirectories for more details.
* [camera](/ai/src/camera/README.md): contains modules interacting with the ZED SDK Python API.
* [data_processing](/ai/src/data_processing/README.md): contains modules that process images and interact with the ZED2i camera.
* [models](/ai/src/models/README.md): contains modules related to a Python class representing a trained model.
* [samples](/ai/src/samples/README.md): contains sample programs that use trained models to detect objects and demonstrate usage of supported APIs.
* [scripts](/ai/src/scripts/README.md): containing runnable scripts that process training data.
These scripts are converted into executable files when the AI package is installed.
They are also used to process training data, balancing the dataset splits, defining crop boxes, etc.
* [training](/ai/src/training/README.md): contains notebooks and scripts that train a model and visualize its metrics.
Trained models and their settings are also included.
* [util](/ai/src/util/README.md): utility functions, mostly related to the file system.

<!-- TOC ignore:true -->
### Image Labelling
Training images are labelled and stored in a web application known as [Roboflow](https://app.roboflow.com/sepb).
Roboflow supports image labelling, image augmentation, image reprocessing, dataset generation, and dataset balancing.
The dataset can be downloaded manually to your local machine or installed programmatically via Roboflow APIs.

Generally, creating a dataset for a model involves the following steps:
1. Create a new project in your workspace and choose the model's purpose.
2. Upload raw image files.
3. Label images.
4. Apply image preprocessing, augmentation and dataset balancing settings.
5. Generate a version of the dataset.

The following articles from Roboflow can be helpful:
* [Getting Started with Roboflow](https://blog.roboflow.com/getting-started-with-roboflow/).
* [How to Train YOLOv5-Classification on a Custom Dataset](https://blog.roboflow.com/train-yolov5-classification-custom-data/).
* [How to Detect Small Objects: A Guide](https://blog.roboflow.com/detect-small-objects/).

<!-- TOC ignore:true -->
### Training
The model is created by retraining an existing YOLOv5 model developed by [Ultralytics](https://docs.ultralytics.com/).
Models are trained via a free Google Collab session running with an NVIDIA GPU.
You may choose to train the model elsewhere, as long as it has an NVIDIA GPU (since CUDA is required by Ultralytics YOLO) and at least 15GB of VRAM.
Training on images at larger sizes (above 800px) or training larger models (Large or Extra Large) consumes more VRAM.

The Jupyter notebook used for training relies on the dataset stored in Roboflow.
After a successful training session, the resulting model (.pt files) and its metrics can be downloaded to your local machine.
The model files (.pt files) are required by all code that uses the model so they must be saved if you intend to use the model for inference.
The Chip Detection model, Tray Detection model, and Case Detection model are included,
see [this README](/ai/src/training/README.md) for more information about dataset and training settings as well as recommended inference settings.

The training process involves the following steps:
1. Ensure you have the Roboflow private API key to download the dataset and know the project name.
	* Both can be accessed by selecting the dataset to be used, then click "Export",
	choose "Show download code", and copy the API key and the project name somewhere secure.
	* Robowflow may append random strings to the project name so it is important that you use the project name shown in the snippet.
	* The API key can also be accessed at [the workspace API settings](https://app.roboflow.com/sepb/settings/api).
2. Connect to a Google Collab session running on a T4 runtime.
3. Run the cells sequentially.
Do not advance to the next cell if the current one fails or has not completed.
The general flow of the notebook is:
	1. Download the dataset by supplying the API key and project name.
	2. Enter the settings to train the model and wait for the training to complete.
	3. View the model metrics.
	4. Optionally download the output folder which contains training metrics and model files.

## Common Issues
This section lists common issues observed during the development of the package and provides fixes or workarounds to avoid them.
Some of these solutions are merely suggestions and are not guaranteed to work.

<!-- TOC ignore:true -->
### AI Packages Not Updated
**Symptoms:**\
The code in the AI package has been modified but the effects do not take place.

**Solution:**\
Ensure that the package with changed code has been reinstalled.
Keep in mind that any existing `build` folder must be deleted before reinstalling,
otherwise the newly updated package is not installed, despite saying so in the terminal.

<!-- TOC ignore:true -->
### Camera Connection Failure
**Symptoms:**\
The code that opens the camera fails with any error code.

**Solution:**\
Ensure that the camera cable is connected and no other application is using the camera.
Unplug the camera USB cable, wait for a few seconds and re-plug it.
If the above solutions do not work, try a different USB port on the machine.

<!-- TOC ignore:true -->
### Correct Detections But Incorrect Positions
**Symptoms:**\
Detection models return correct bounding boxes but the numeric positions used by the cobot is not.
This can occur when the crop boxes are not sufficiently "tight" or correctly aligned.
The bounding box-to-position code has different criteria for different types of models and should be adhered to.
These requirements can be found in the [data_processing](/ai/src/data_processing/README.md) module.

**Solution:**\
Verify that the crop boxes are correctly aligned.
Keep in mind that any physical adjustments to the camera may alter its angle, leading to inaccuracies in existing crop boxes.
It is strongly recommended to assess the model's performance using sample scripts found in [samples](/ai/src/samples/README.md) before using it to guide the cobot.

<!-- TOC ignore:true -->
### Google Collab Connection Failure
**Symptoms:**\
Google Collab refuses to open a new session.

**Solution:**\
Unfortunately, using free sessions from Google Collab limits your usage time.
The time limit remains unclear, but it is suggested that Google Collab incrementally decrease your allocated time if you keep a session running for too long.
The solution is to either use the paid version, or limit your usage time as much as possible.

<!-- TOC ignore:true -->
### Images Not Displayed
**Symptoms:**\
The function `show_image()` in the [image_processing.py](/ai/src/data_processing/image_processing.py) module does not display images when called.
This is accompanied by warning messages that look like below in the terminal:\
`eog: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`

**Solution:**\
This is most likely caused by the `GTK_PATH` environment variable in VSCode's integrated terminals.
The following solutions can be used:
* Unset that variable, either by calling `unset GTK_PATH` or pasting this to the `settings.json` file of VSCode:
	```json
	"terminal.integrated.env.linux": {
		"GTK_PATH": ""
	}
	```
* Run the code in an external terminal.

Note that even when the images are successfully displayed, warning messages might appear in the terminal but can be ignored.
For more information,
see [this StackOverflow thread](https://askubuntu.com/questions/1462295/ubuntu-22-04-both-eye-of-gnome-and-gimp-failing-with-undefined-symbol-error).

<!-- TOC ignore:true -->
### pyzed.sl Import Failure
**Symptoms:**\
Running code that uses the camera fails with the error:
```bash
	import pyzed.sl as sl
ImportError: DLL load failed while importing sl: The specified module could not be found.
```

**Solution:**\
Ensure that the ZED SDK is installed correctly by checking the following steps:
* Your machine has CUDA which requires a NVIDIA GPU.
* The ZED SDK is installed.
* The Python API is installed via the script `get_python_api.py`.
See the [Dependencies](#dependencies-1) section for more details.

If all of the above steps fail, refer to these online threads or raise an issue with the ZED SDK maintainers themselves.
Note that none of these issues have been encountered during development after following the above steps:
* [GitHub Issue: \[ImportError: No module named 'pyzed.sl'\]](https://github.com/stereolabs/zed-tensorflow/issues/10)
* [GitHub Issue: \[cannot import pyzed.sl\]](https://github.com/stereolabs/zed-python-api/issues/78)
* [GitHub Issue: \[Error: import pyzed.sl as sl\]](https://github.com/stereolabs/zed-sdk/issues/358)
* [Stereo Labs Community Thread](https://community.stereolabs.com/t/importerror-no-module-named-pyzed-sl-as-sl/2467)

<div class="page"/><!-- page break -->

# Camera APIs
## camera_capture.py
Contains functions that controls the ZED camera programmatically.\
Functionalities range from opening the camera, applying configured settings and taking images.

<!-- TOC ignore:true -->
## camera_lens.py
Contains an enum representing the camera logical lens.
If the camera is flipped (by setting turning on Flip Mode), the physical left lens becomes the logical right lens and vice versa.

## Capturing Images via Terminal
To capture data with the ZED camera we are going to use the SDK default files, and the example code found in GitHub.

From the SDK we need:
* `ZED_Explorer`
* `ZED_SVO_Editor`

From the [ZED_EXAMPLES](https://github.com/stereolabs/zed-examples) we need:
```bash
samples/recording/export/svo/python
```

So, we will leave this folder empty.\
By default, the binary files can be found in `usr/local/bin/` on a Desktop, and in `/usr/local/zed/tools` in the Jetson Nano.

When executing these Bash scripts, the paths with `$` should be changed to the appropriate directory names, these can also be defined in a program prior to execution.

<!-- TOC ignore:true -->
### How to Capture Data
Capturing 3 seconds and 4 frames extra.
3 seconds to settle down the auto-exposure:
```bash
ZED_Explorer -r HD2K -f 15 -l 50 --cpm -m 0 -o $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP-autoexp.svo
```

Cutting the first 3 seconds due to auto-exposure:
```bash
ZED_SVO_Editor -cut $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP-autoexp.svo -s 45 -e 49 $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP.svo
```

Get `.png` images:
```bash
Zed_SVO_Export.py --mode 2 --input_svo_path $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP.svo --output_dir_path $PATH_TO_DATA/$SCENE_NUMBER/ZED/
```

<div class="page"/><!-- page break -->

# Data Processing APIs
## case_position.py
Contains function that converts the bounding boxes of cases to position from 1 to 17, with 1 being at the bottom of the case rack.

**Important**
* This function requires the case image to be cropped such that the image bottom aligns with the bottom of the horizontal T-slot bar.
The height of the image is around 514px and this limit is not flexible.
* Before attempting to pick the case with the cobot, it is important to test the model and this code on every position on the rack.
The sample programs found in [/ai/src/samples/](/ai/src/samples/README.md) can be helpful in this regard.

## image_processing.py
Contains functions that modifies images.
Functionalities involve cropping, drawing rectangular crop boxes on images with labels and displaying them.

## tray_position.py
Translate bounding boxes of tray into a cobot move.
Supported cobot moves include:
* Move tray 1 to assembly
* Move tray 2 to assembly
* Move tray 1 from assembly
* Move tray 2 from assembly
* Start loading items on tray 1
* Start loading items on tray 2
* Continue loading items on partially loaded tray 1
* Continue loading items on partially loaded tray 2

**Important**:
* The Tray Detection model does not detect individual items on a tray, it simply classifies trays into 3 categories: empty, partially full, and full.
The "Continue Loading Trays" move is intended to signal that the tray is partially full, which may or not may not be an error depending on the state of the cobot.
* The position conversion code assumes the following:
	* There are only 2 trays in 3 possible positions: Assembly, Tray 1 and Tray 2, with Assembly being on the left, Tray 1 and Tray 2 being on the right.
	* The image containing the tray should be roughly 600px in height and 700px in width,
	with Assembly tray on the left half, and Trays 1 and 2 vertically aligned in the right half.

## chip_position.py
Translate bounding boxes of chips into positions defined by the cobot.
These positions are numbered from 1 to 48, with 1 being the top left and 48 being the bottom right.
The positions increase by columns than rows.

**Important**
* The code requires the prediction to be made on a image with a height of around 230px and width of around 320px.
* The position is calculated using the center point of individual bounding boxes and checking them against a range of valid values.
The center point is used instead of the entire bounding box as the box can appear to be horizontally shifted when viewing a chip from an angle.

<div class="page"/><!-- page break -->

# Model APIs
## detected_object.py
Dataclass that holds the class index, confidence and crop box of detected objects.
The class name can be retrieved using the index and `ObjectDetectionModel.classes`.

## object_detection_model.py
Wrapper around the YOLO class from Ultralytics, created to narrow the functionalities and interface to fit this project.\
Requires a YAML configuration file to initialise the model and its parameters.
This class supports an option to save the image output and/or display it in a window.
To find more details about the configuration YAML file, see [samples/README.md](/ai/src/samples/README.md)

<div class="page"/><!-- page break -->

# Sample object detection programs
## basic_sample.py
<!-- TOC ignore:true -->
### Purpose
Used to demonstrate usage of a YOLO model class and quickly test trained models without involving the camera.

<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
User can choose from the 2 options:
* **Option 0**:
	* Model file (.pt file, not to be confused with yaml configuration file) selected via file dialogue.
	* Cropping is not supported.
* **Option 1**:
	* Designed to be used with a YAML configuration file selected by the user via a file dialogue.
	* Cropping, model files and camera settings can be set in the configuration file.

## camera_sample.py
<!-- TOC ignore:true -->
### Purpose
Runs inference using a trained model and an image captured by the ZED camera.
All configurations for the model, crop box, and camera settings are specified in the YAML file, which is selected via file dialogue.

The recommended usage is to test a newly trained model with this script and verify the results before using it on the cobot.
The sample also demonstrates the use of APIs offered by this package.

<!-- TOC ignore:true -->
### Usage
The user can choose from 3 options:
* **Option 0**: Run chip detection model.
* **Option 1**: Run tray detection model.
* **Option 2**: Run case detection model.

## sample_config.yaml
Sample configuration yaml file, containing all configurable properties with example values.
This file is intended as an example only, its values might not provide the best performance and should be tailored to your specific requirements.

Depending on your code, not all properties declared in this files are accessed.
For example, if you plan to run inference using saved images rather than the camera, the `camera` portion can be omitted.
However, this is not guaranteed to be error-free so all properties should be declared, even with dummy values.

To find more information about default values for the model configuration and camera settings from their authors, visit the following pages:
* [Ultralytics Inference Arguments](https://docs.ultralytics.com/modes/predict/#inference-arguments)
* [ZED SDK Python API Reference](https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1VIDEO__SETTINGS.html)

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.

<div class="page"/><!-- page break -->

# Training Data Processing Scripts
## dataset_balancer.py
<!-- TOC ignore:true -->
### Purpose
Robowflow asks for training/test/validation ratio,
which is invalid after the augmentation process because the training set size is increased by a factor (2 or 3 when using a free tier).
This script calculates the amount of images used for training, testing and validation accounting for extra images from the augmentation process.

<!-- TOC ignore:true -->
### Usage
The user is asked to enter the dataset size, desired training set ratio after augmentation and
the factor by which the training set will be multiplied by the augmentation process in the terminal.
The program prints the recommended size for the train set, test set, and validation set to meet the desired ratio.

## copy_interval.py
<!-- TOC ignore:true -->
### Purpose
Copy files from a folder with a user-defined interval.\
This script is useful if you collect data by taking a video and export images (frames) from it.
The resulting image set contains many duplicates which can be trimmed down by selecting images at an interval.

<!-- TOC ignore:true -->
### Usage
The user is asked to select the source folder, destination folder and the interval by which an image from the source folder is copied to the destination one.

## define_crop.py
<!-- TOC ignore:true -->
### Purpose
Defines a crop box and applies it to all selected files.
Since the AI models rely on image that is cropped to a specific region of interest, cropping images are required for both training and evaluation.

<!-- TOC ignore:true -->
### Usage
The user will be asked to define a crop box and apply it to select images.
The crop box coordinates are printed in the terminal.
The following options are supported:
1. Select an image to define a crop box, then apply it to a set of selected images and save them to a folder.
2. Enter the crop box coordinates into the console, apply it to a set of images and save them to a folder.
3. Capture an image from the ZED camera and use it to define a crop box, then apply it to a set of images and save them to a folder.

## random_crop.py
<!-- TOC ignore:true -->
### Purpose
Creates a specified number of random crops from an image with the specified dimensions.

Primarily used for generating background images to diversify datasets for models relying on static cropping.
Without these additional images, the model may produce numerous false positive predictions when applied on an image outside the cropped area.
Including these extra background images enhances the models' accuracy and resilience when used on images that were trained with static cropping.

<!-- TOC ignore:true -->
### Usage
The user will be asked to supply the source image, enter the coordinates of the crop box, the number of images to generate, and the destination folder.

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.

<div class="page"/><!-- page break -->

# Training
## object_detection_training.ipynb
<!-- TOC ignore:true -->
### Purpose
Notebook that downloads the dataset from Roboflow and train the YOLO model.

<!-- TOC ignore:true -->
### Usage
The user is allowed to input various parameters such as epochs, image size, batch size, frozen layers, etc.
If the notebook is run on Google Collab, the result folder of every training session can be downloaded as a zip file.

## plot_ultralytics_results.py
<!-- TOC ignore:true -->
### Purpose
Training the YOLO model from Ultralytics will produce several images showing the model's metrics.\
However, YOLO does not support displaying overlays of different metrics, such as showing the training curve and validation curve in the same graph.\
This script is created to address this issue by plotting them based on the data retrieved from the `results.csv` file produced after every training session.

<!-- TOC ignore:true -->
### Usage
The user can select a `results.csv` file generated after a YOLO model is trained by Ultralytics.
If the `object_detection_training.ipynb` is used to train the model, the `results.csv` file is included in the output.

## trained
<!-- TOC ignore:true -->
### Purpose
This directory contains trained models in the form of PyTorch files.
Full path to these files must be included in the configuration YAML so the model can be found and used to run inference.

<!-- TOC ignore:true -->
### Usage
This section contains details of how these models are trained.
For best results, the size of images used in training should not be significantly different from those used in inference.
In training, the rectangular mode should be turned ON.
This setting converts a rectangular image into a square image with the size being the longer dimension by filling out the space with a solid color.
In inference, rectangular mode is turned on by default.
See [GitHub comment 1](https://github.com/ultralytics/yolov5/issues/2009#issuecomment-766147324) and
[GitHub comment 2](https://github.com/ultralytics/yolov5/issues/2009#issuecomment-765557040) for more details.

The following section includes the settings used to train these models on Roboflow and recommended settings when running inference with them.

**detect_chip.pt:**
* Purpose: detect blue chips in red slots and yellow chips in white slots
* Class(es): `chips`
* Training settings:
	* Epochs: 100
	* Image size (rectangular mode turned ON): 416px
	* Dataset:
		* Version: 12
		* Size: 1194 images
		* Preprocessing:
			* Auto-Orient: Applied
		* Augmentation:
			* Flip: Horizontal, Vertical
			* Rotation: Between -30&deg; and +30&deg;
			* Shear: &plusmn;15&deg; Horizontal, &plusmn;15&deg; Vertical
			* Saturation: Between -10% and +10%
			* Brightness: Between -5% and +5%
			* Mosaic: Applied
			* Bounding Box: Flip: Horizontal, Vertical
* Recommended inference settings:
	* Image size: around 384px (W) x 256px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5

**detect_case.pt:**
* Purpose: detect blue chips in red slots and yellow chips in white slots
* Class(es): `case`
* Training settings:
	* Epochs: 80
	* Image size (rectangular mode turned ON): 544px
	* Dataset:
		* Version: 1
		* Size: 915 images
		* Preprocessing:
			* Auto-orient: applied
		* Augmentation:
			* Saturation: Between -20% and +20%
			* Brightness: Between -10% and +10%
			* Exposure: Between -10% and +10%
* Recommended inference settings:
	* Image size: around 117px (W) x 514px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5

**detect_tray.pt:**
* Purpose: detect white trays
* Class(es): `Full`, `Empty`, `partially Full` (inconsistent casing caused by typo when labelling the data)
* Training settings:
	* Epochs: 130
	* Image size (rectangular mode turned ON): 704px
	* Dataset:
		* Version: 6
		* Size: 594 images
		* Preprocessing:
			* Auto-orient: applied
		* Augmentation:
			* Flip: Horizontal, Vertical
			* Hue: Between -74&deg; and +74&deg;
			* Saturation: Between -25% and +25%
			* Exposure: Between -25% and +25%
			* Bounding Box: 90&deg; Rotate: Clockwise, Counter-Clockwise, Upside Down
			* Bounding Box: Brightness: Between -15% and +15%
* Recommended inference settings:
	* Image sizes: around 661px (W) x 568px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5

<div class="page"/><!-- page break -->

# Util APIs
## file_dialog.py
Provides functions to interact with the file dialogues.
Instructions for the user are typically placed in the file window name.
The file dialogue might fail to open in the foreground, but this is a very rare error.

## file_reader.py
Provides functions to read YAML files.
