from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  namespace = LaunchConfiguration('namespace')
  ip =        LaunchConfiguration('ip')
  port =      LaunchConfiguration('port')
  gripper =   LaunchConfiguration('gripper')
  changer_addr =   LaunchConfiguration('changer_addr')
  dummy =     LaunchConfiguration('dummy')

  ns_arg =      DeclareLaunchArgument('namespace', default_value='')
  ip_arg =      DeclareLaunchArgument("ip",        default_value="192.168.1.1")
  port_arg =    DeclareLaunchArgument("port",      default_value="502")
  gripper_arg = DeclareLaunchArgument("gripper",   default_value="rg6")
  changer_addr_arg = DeclareLaunchArgument("changer_addr",   default_value="65")
  dummy_arg =   DeclareLaunchArgument("dummy",     default_value="false")

  status = Node(
    name='OnRobotRGStatusListener',
    package='onrobot_rg_control',
    namespace=namespace,
    executable='OnRobotRGStatusListener.py',
    parameters=[{
      'gripper' : gripper,
    }]
  )

  comms = Node(
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


  return LaunchDescription([
    ns_arg, ip_arg, port_arg, gripper_arg, changer_addr_arg, dummy_arg,
    status, comms])