# Import the ROS2 Python library for communication.
import rclpy
# Import the CobotMovement class from the 'cobot_movement' module.
from nodes.gripper_server import GripperServer


def main(args=None):
	# Initialize the ROS2 Python client library.
	rclpy.init(args=args)

	# Create an instance of the GripperNode class.
	gripper_server = GripperServer()

	# Enter the ROS2 event loop and spin the publisher node.
	rclpy.spin(gripper_server)

	# Destroy the publisher node.
	gripper_server.destroy_node()

	# Shutdown the ROS2 client library when done.
	rclpy.shutdown()


if __name__ == "__main__":
	# Call the main function to start the ROS2 node.
	main()
