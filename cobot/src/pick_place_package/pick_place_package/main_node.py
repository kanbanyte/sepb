# Import the rclpy library for working with ROS 2 Python.
import rclpy
from nodes.main_action_client import MainActionClient

def main(args=None):
	rclpy.init(args=args)
	action_client = MainActionClient()

	while True:
		result = action_client.send_pick_place_request()
		if not result.result.task_successful and input("Request failed, exit loop? (y/n): ") == 'y':
			print("Client exiting loop...")
			break


if __name__ == '__main__':
	main()
