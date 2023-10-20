#!/bin/bash

source $HOME/Documents/repos/sepb/bash/variables.sh

# Ping cobot twice
function ping_cobot() {
	ping -c 2 172.21.0.121
}

# Initialise ROS2
function init_ros2() {
	source /opt/ros/humble/setup.bash
	export ROS_DOMAIN_ID=10
}

# Launch cobot driver
function launch_driver() {
	ros2 launch ur_robot_driver ur_control.launch.py \
	ur_type:=ur5e robot_ip:=172.21.0.121 launch_rviz:=false use_tool_communication:=true kinematics_config:=ur5e_fof_calibration.yaml
}

# Establish cobot connection
function connect_cobot() {
	ping_cobot
	init_ros2
	launch_driver
}

#region Bootup
# If not done manually, load the program ROS.urp which allows remote control.
# This can be done manually, or via command line.
# If done via command line makes the driver crash, so restart the robot driver.
function load_ros_file() {
	ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load filename:\ \'ROS.urp\'
}

# Power on the robot motors.
function power_on_cobot() {
	ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger
}

# Release break.
function release_brake() {
	ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
}

# IMPORTANT: Stop and start the program if it is already running.
# This was causing the robot not moving for no reason.
# Make sure you stop and start the program if you are restarting the driver.
function play_cobot() {
	ros2 service call /dashboard_client/play std_srvs/srv/Trigger
}

# Pause cobot
function stop_cobot() {
	ros2 service call /dashboard_client/stop std_srvs/srv/Trigger
}

# Check controllers, switch to "scaled_joint_trajectory_controller".
# Check again after switching JIC
function switch_controllers() {
	ros2 control switch_controllers --activate scaled_joint_trajectory_controller
}

# Launch MoveIt with Rviz.
# IMPORTANT: If the Rviz robot position is not like the real robot, something has gone wrong.
# So, restart the driver, stop, play, switch controller, MoveIt
function launch_rviz() {
	ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
}
#endregion Bootup

#region Package
# Rebuild cobot package
function rebuild_cobot() {
	rm -r $COBOT_BUILD_DIR
	rm -r $COBOT_INSTALL_DIR
	rm -r $COBOT_LOG_DIR
	cd $COBOT_PATH
	colcon build
	source install/local_setup.bash
	rosdep install -i --from-path src --rosdistro humble -y
}

# Launch cobot package
function launch_cobot() {
	ros2 launch pick_place_package pick_place_launch.py
}

# Start cobot package
function start_cobot() {
	init_ros2
	rebuild_cobot
	launch_cobot
}
#endregion Package

#region Nodes
# Test Camera
function run_camera() {
	ros2 run pick_place_package camera_node
}

# Test cobot
function run_cobot() {
	ros2 run pick_place_package cobot_node
}

# Test Gripper
function run_gripper() {
	ros2 run pick_place_package gripper_node
}

# Test package
function run_main() {
	ros2 run pick_place_package main_node
}
#endregion Nodes

#region Services
# Case Service Call
function service_call_case() {
	ros2 service call /case pick_place_interfaces/srv/PickPlaceService "{detect: True}"
}

# Chip Service Call
function service_call_chip() {
	ros2 service call /chip pick_place_interfaces/srv/PickPlaceService "{detect: True}"
}

# Tray Service Call
function service_call_tray() {
	ros2 service call /tray pick_place_interfaces/srv/PickPlaceService "{detect: True}"
}

# Send a Goal to Perform Pick Place Action
function send_goal_action() {
	ros2 action send_goal /perform_pick_place pick_place_interfaces/action/PickPlaceAction "{perform_pick_place: True}"
}
#endregion Services

#region Gripper I/O
# Load Gripper file for Tests
function load_gripper_file() {
	ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load filename:\ \'gripper_test.urp\'
}

# Completely Close Gripper
function close_fully() {
	ros2 service call /gripper_service ur_msgs/srv/SetIO "{fun: 1, pin: 2, state: 1.0}"
}

# Completely Open Gripper
function open_fully() {
	ros2 service call /gripper_service ur_msgs/srv/SetIO "{fun: 1, pin: 1, state: 1.0}"
}

# Open Gripper for Tray
function open_partly() {
	ros2 service call /gripper_service ur_msgs/srv/SetIO "{fun: 1, pin: 3, state: 1.0}"
}

# Open Gripper for Battery
function open_grip() {
	ros2 service call /gripper_service ur_msgs/srv/SetIO "{fun: 1, pin: 4, state: 1.0}"
}

# Pinch Gripper for Chip
function pinch_grip() {
	ros2 service call /gripper_service ur_msgs/srv/SetIO "{fun: 1, pin: 5, state: 1.0}"
}
#endregion Gripper I/O
