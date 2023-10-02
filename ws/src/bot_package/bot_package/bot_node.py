# Import the ROS2 Python library for communication.
import rclpy
# Import the PublisherJointTrajectory class from the 'bot_move' module.
from nodes.bot_move import PublisherJointTrajectory
from nodes.camera_node import CameraServer


def main(args=None):
	# Initialize the ROS2 Python client library.
	rclpy.init(args=args)

	# Create an instance of the PublisherJointTrajectory class.
	# publisher_joint_trajectory = PublisherJointTrajectory()
	camera_server = CameraServer()

	# Enter the ROS2 event loop and spin the publisher node.
	# rclpy.spin(publisher_joint_trajectory)
	rclpy.spin(camera_server)


	# Destroy the publisher node.
	# publisher_joint_trajectory.destroy_node()
	camera_server.destroy_node()

	# Shutdown the ROS2 client library when done.
	rclpy.shutdown()


if __name__ == "__main__":
	# Call the main function to start the ROS2 node.
	main()
