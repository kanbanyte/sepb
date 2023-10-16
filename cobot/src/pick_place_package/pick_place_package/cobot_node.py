# Import the ROS2 Python library for communication.
import rclpy
# Import the CobotMovement class from the 'cobot_movement' module.
from nodes.cobot_movement import CobotMovement


def main(args=None):
	# Initialize the ROS2 Python client library.
	rclpy.init(args=args)

	# Create an instance of the CobotMovement class.
	publisher_joint_trajectory = CobotMovement()

	# Enter the ROS2 event loop and spin the publisher node.
	rclpy.spin(publisher_joint_trajectory)

	# Destroy the publisher node.
	# publisher_joint_trajectory.destroy_node()

	# Shutdown the ROS2 client library when done.
	# rclpy.shutdown()


if __name__ == "__main__":
	# Call the main function to start the ROS2 node.
	main()
