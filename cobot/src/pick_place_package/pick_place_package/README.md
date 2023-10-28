
<!-- TOC ignore:true -->
# Pick and Place Packages
**Table of Contents**
<!-- TOC -->

* [camera_node.py](#camera_nodepy)
* [cobot_node.py](#cobot_nodepy)
* [gripper_node.py](#gripper_nodepy)
* [main_node.py](#main_nodepy)

<!-- /TOC -->

## camera_node.py
<!-- TOC ignore:true -->
### Purpose
Camera node is a ROS2 Node that supplies the position of objects when requested.\
This node is built upon the class [CameraServer](../nodes/camera_server.py) and must be running before the Cobot Action Server receives a request.

<!-- TOC ignore:true -->
### Usage
The camera node is rebuilt and started along with the Cobot Action Server if the system is started with the command `start_cobot`.

## cobot_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [CobotActionServer](../nodes/cobot_movement.py) class, to act as an action server retrieving requests to perform a pick-and-place task.

<!-- TOC ignore:true -->
### Usage
This node is rebuilt and started with all its dependencies if the system is started with the command `start_cobot`.

## gripper_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [GripperServer](../nodes/gripper_server.py) class, to act as a server that opens or closes the gripper upon a request.\
This node is used by the Cobot Action Server to grab items before moving them.

<!-- TOC ignore:true -->
### Usage
The gripper node is rebuilt and started along with the Cobot Action Server if the system is started with the command `start_cobot`.

## main_node.py
<!-- TOC ignore:true -->
### Purpose
This node runs the [MainActionClient](../nodes/main_action_client.py) class, to act as a client to the Cobot Movement Action Server.\
It sends requests to the action server in an infinite loop to keep the cobot running constantly.

<!-- TOC ignore:true -->
### Usage
The client node is rebuilt and started via the command `run_main`.\
If a request fails, the user will be asked whether to continue the loop in the terminal.
