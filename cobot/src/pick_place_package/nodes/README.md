
<!-- TOC ignore:true -->
# Node APIs
**Table of Contents**
<!-- TOC -->

* [camera_server.py](#camera_serverpy)
* [cobot_methods.py](#cobot_methodspy)
* [cobot_movement.py](#cobot_movementpy)
* [gripper_server.py](#gripper_serverpy)
* [main_action_client.py](#main_action_clientpy)
* [read_methods.py](#read_methodspy)

<!-- /TOC -->

## camera_server.py
This class is a node acting as a server that provides positions of items upon request.
Object positions or tray movements can be requested via three different services: tray service, case service, and chip service, all of which share the same service interface.
The returned integer signal can be interpreted differently based on the called service:
* Chip service: returned signal is a number from 1 to 48 if a chip is detected in these positions or -1 if none is detected.
* Case service: returned signal is a number from 1 to 17 if a case is detected in these positions or -1 if none is detected.
* Tray service: returned signal is the value from the enum `CobotMovement` from the AI package.

## cobot_methods.py
This modules contains functions that populates the trajectories of the cobot.
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


## main_action_client.py
This class acts as an action client node which sends pick-and-place request to the Cobot Movement Action Server and waits until that request is complete.
The result and status of the request are printed to the terminal once the request is completed.
Feedback detailing the cobot movements during the process are also provided.

## read_methods.py
