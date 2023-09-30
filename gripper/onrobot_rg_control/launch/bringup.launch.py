from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	namespace = LaunchConfiguration('namespace')
	ip = LaunchConfiguration('ip')
	port = LaunchConfiguration('port')
	gripper = LaunchConfiguration('gripper')
	changer_addr = LaunchConfiguration('changer_addr')
	dummy = LaunchConfiguration('dummy')

	return LaunchDescription([
		DeclareLaunchArgument('namespace', default_value=''),
		DeclareLaunchArgument("ip", default_value="172.21.0.121"),
		DeclareLaunchArgument("port", default_value="54321"),
		DeclareLaunchArgument("gripper", default_value="rg2"),
		DeclareLaunchArgument("changer_addr", default_value="65"),
		DeclareLaunchArgument("dummy", default_value="false"),
		Node(
			name='OnRobotRGStatusListener',
			package='onrobot_rg_control',
			namespace=namespace,
			executable='OnRobotRGStatusListener.py',
			parameters=[{'gripper' : gripper,}]
		),
		Node(
			name='OnRobotRGTcpNode',
			package='onrobot_rg_control',
			namespace=namespace,
			executable='OnRobotRGTcpNode.py',
			parameters=[{
				'ip' : ip,
				'port' : port,
				'gripper' : gripper,
				'changer_addr':changer_addr,
				'dummy' : dummy,
			}]
		)
	])
