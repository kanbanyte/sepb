#!/bin/bash

#region Package
# Initialise ROS2
function init_ros2() {
	source /opt/ros/humble/setup.bash
	export ROS_DOMAIN_ID=10
}

# Rebuild cobot package
function rebuild_cobot() {
	rm -r /home/cobot/Documents/repos/sepb/cobot/build
	rm -r /home/cobot/Documents/repos/sepb/cobot/install
	rm -r /home/cobot/Documents/repos/sepb/cobot/log
	cd /home/cobot/Documents/repos/sepb/cobot
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
