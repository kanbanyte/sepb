from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	# Define the path to a YAML configuration file for position goals
	position_goals = PathJoinSubstitution([FindPackageShare("bot_package"), "config", "bot_config.yaml"])

	# Create a LaunchDescription, which represents the launch file
	return LaunchDescription([
		Node(
			package="bot_package",
			executable="bot_node",
			name="bot_node",
			parameters=[position_goals],
			output="screen",
		),
		Node(
			package="bot_package",
			executable="camera_node",
			name="camera_node",
			parameters=[position_goals],
			output="screen",
		),
		# Node(
		# 	package="bot_package",
		# 	executable="gripper_node",
		# 	name="gripper_node",
		# 	parameters=[position_goals],
		# 	output="screen",
		# )
	])
