from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	position_goals = PathJoinSubstitution([FindPackageShare("bot_package"), "config", "bot_config.yaml"])

	return LaunchDescription([
		Node(
			package="bot_package",
			executable="bot_node",
			name="bot_positions",
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
		Node(
			package="bot_package",
			executable="main_node",
			name="main_node",
			parameters=[position_goals],
			output="screen",
		)
	])
