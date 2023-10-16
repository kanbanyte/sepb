# Steps To Produce
The following demonstrates terminal commands and their outputs so far.\
Note that a few might still be WIP.

## Case Service Call
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

## Chip Service Call
```bash
ros2 service call /chip pick_place_interfaces/srv/PickPlaceService "{detect: True}"
```

> Output.

```txt
requester: making request: pick_place_interfaces.srv.PickPlaceService_Request(detect=True)

response:
pick_place_interfaces.srv.PickPlaceService_Response(signal=7)
```

## Tray Service Call
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

## List All Services
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

## List All Nodes
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

## List All Actions
```bash
ros2 action list
```

> Output.

```txt
/perform_pick_place
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

## Send a Goal to Perform Pick Place Action
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
