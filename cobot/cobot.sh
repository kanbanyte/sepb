#!/bin/bash

function init_ros2() {
	source /opt/ros/humble/setup.bash
	export ROS_DOMAIN_ID=10
}

function rebuild_project() {
	rm -r /home/cobot/Documents/repos/sepb/cobot/build
	rm -r /home/cobot/Documents/repos/sepb/cobot/install
	rm -r /home/cobot/Documents/repos/sepb/cobot/log
	cd /home/cobot/Documents/repos/sepb/cobot
	colcon build
	source install/local_setup.bash
	rosdep install -i --from-path src --rosdistro humble -y
}

function launch_cobot() {
	ros2 launch pick_place_package pick_place_launch.py
}

function start_project() {
	init_ros2
	rebuild_project
	launch_cobot
}
