# Import the rclpy library for working with ROS 2 Python.
import rclpy
# Import the MainActionClient class from the 'main_action_client' module.
from nodes.main_action_client import MainActionClient
import asyncio


async def do_pick_place_async(action_client):
	while True:
		# future = await action_client.send_goal(True)
		# rclpy.spin_until_future_complete(action_client, future)
		await action_client.send_goal(True)
		rclpy.spin_once(action_client)
		print("here")

def main(args=None):
	# Initialize the ROS 2 Python node.
	rclpy.init(args=args)

	# Create an instance of the MainActionClient class, which is responsible for sending action goals.
	action_client = MainActionClient()

	# loop = asyncio.get_event_loop()

	action_client.send_goal(True)

	rclpy.spin(action_client)

	# asyncio.run(do_pick_place_async(action_client))

	print("hello")


if __name__ == '__main__':
	# Call the main function when this script is executed directly.
	main()
