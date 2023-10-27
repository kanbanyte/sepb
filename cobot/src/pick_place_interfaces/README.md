
<!-- TOC ignore:true -->
# Pick and Place Interfaces
<!-- TOC ignore:true -->
## Overview
The pick and place interfaces package contains custom interfaces for use within [`pick_place_package`](/cobot/src/pick_place_package/README.md)
that allows each node to communicate with each other appropriately.
The package contains two directories:
* `action` which contains the action file for communicating between [`main_node`](/cobot/src/pick_place_package/pick_place_package/main_node.py) and
[`cobot_node`](/cobot/src/pick_place_package/pick_place_package/cobot_node.py).
* `srv` which contains the service file for communicating between [`cobot_node`](/cobot/src/pick_place_package/pick_place_package/cobot_node.py) and
[`camera_node`](/cobot/src/pick_place_package/pick_place_package/camera_node.py).

**Table of Contents**
<!-- TOC -->

* [PickPlaceAction.action](#PickPlaceActionaction)
* [PickPlaceService.srv](#PickPlaceServicesrv)

<!-- /TOC -->

## PickPlaceAction.action
<!-- TOC ignore:true -->
### Purpose
`PickPlaceAction.action` is an action file that allows `main_node` to initiate the pick and place task.
It sends a boolean as a goal that is used to start the task, it returns the name of the current movement as feedback,
and returns a boolean for whether the task was successful or not.

<!-- TOC ignore:true -->
### Usage
It is used in both `main_node` and `cobot_node`.
`main_node` sends a goal to the `cobot_node` which then returns the name of the current trajectory as feedback before returning the task's success.

## PickPlaceService.srv
<!-- TOC ignore:true -->
### Purpose
`PickPlaceService.srv` is a service file for use with the `chip`, `case`, and `tray` services as defined in
[`camera_server.py`](/cobot/src/pick_place_package/nodes/camera_server.py).
It uses a boolean to request the camera node to detect the relevant objects,
and returns an `int64` as a response corresponding to a chip position, case position, or tray movement.

<!-- TOC ignore:true -->
### Usage
It is used in both `camera_node` and `cobot_node`.
`cobot_node` requests a case, chip, or tray position from `camera_node` which then returns the signal as a response.
