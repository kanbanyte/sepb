from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	# Define the path to a YAML configuration file for position goals
	position_goals = PathJoinSubstitution([FindPackageShare("pick_place_package"), "config", "cobot_config.yaml"])

	# Create a LaunchDescription, which represents the launch file
	return LaunchDescription([
		Node(
			package="pick_place_package",
			executable="gripper_node",
			name="gripper_node",
			parameters=[position_goals],
			output="screen",
		),
		Node(
			package="pick_place_package",
			executable="camera_node",
			name="camera_node",
			parameters=[position_goals],
			output="screen",
		),
		Node(
			package="pick_place_package",
			executable="cobot_node",
			name="cobot_node",
			parameters=[position_goals],
			output="screen",
		),
		TimerAction(
			period=20.0,
			actions=[
				Node(
					package="pick_place_package",
					executable="main_node",
					name="main_node",
					parameters=[position_goals],
					output="screen",
				)
			]
		),
	])
