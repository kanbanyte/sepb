# Import the rclpy library for working with ROS 2 Python.
import rclpy
# Import the MainActionClient class from the 'main_action_client' module.
from nodes.main_action_client import MainActionClient


def main(args=None):
	# Initialize the ROS 2 Python node.
	rclpy.init(args=args)

	# Create an instance of the MainActionClient class, which is responsible for sending action goals.
	action_client = MainActionClient()

	# Send a goal using the action client and store the returned Future object.
	future = action_client.send_goal(True)

	# Spin until the specified future is complete, which involves handling callbacks.
	rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
	# Call the main function when this script is executed directly.
	main()
