from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	namespace = LaunchConfiguration('namespace')
	gripper = LaunchConfiguration('gripper')
	changer_addr = LaunchConfiguration('changer_addr')
	dummy = LaunchConfiguration('dummy')

	ns_arg = DeclareLaunchArgument('namespace', default_value='')
	gripper_arg = DeclareLaunchArgument("gripper", default_value="rg2")
	changer_addr_arg = DeclareLaunchArgument("changer_addr", default_value="65")
	dummy_arg = DeclareLaunchArgument("dummy", default_value="false")

	status = Node(
		name='OnRobotRGStatusListener',
		package='onrobot_rg_control',
		namespace=namespace,
		executable='OnRobotRGStatusListener.py',
		parameters=[{
			'gripper' : gripper,
		}]
	)

	return LaunchDescription([
		ns_arg,
		gripper_arg,
		changer_addr_arg,
		dummy_arg,
		status,
		comms
	])
