# Import the rclpy library for working with ROS 2 Python.
import rclpy
# Import the MainActionClient class from the 'main_action_client' module.
from nodes.main_action_client import MainActionClient

def test_0(action_client):
	while True:
		future = action_client.send_goal_async()
		rclpy.spin_until_future_complete(action_client, future)

def test_1(action_client):
	future = action_client.send_goal_async()
	# TODO: if one of them returns something other than finished, loop on that value
	print(f"Status: {future.status}")
	print(f"Done: {future.done()}")
	print(f"Result: {future.result()}")
	rclpy.spin_until_future_complete(action_client, future)

def test_2(action_client):
	future = action_client.send_goal(True)
	while not future.done():
		print("Waiting for action request to complete")
		# TODO: if the loop works, but only execute once, try replacing `spin_until_future_complete`
		rclpy.spin_until_future_complete(action_client, future)

import threading
import time
FlagLock = threading.Lock()
Flag = False
def set_done_flag(future):
	print("Flag set")
	with FlagLock:
		global Flag
		Flag = True

def test_3(action_client):
	global Flag
	while True:
		with FlagLock:
			print("Flag reset")
			Flag = False

		future = action_client.send_goal_async()
		future.add_done_callback(set_done_flag)

		while True:
			FlagLock.acquire(blocking=True, timeout=2)
			if Flag is True:
				FlagLock.release()
				print("Flag was set, request complete, exiting loop...")
				break
			else:
				FlagLock.release()
				print("Sleeping for 2 secs")
				time.sleep(2)

				rclpy.spin_until_future_complete(action_client, future)
				continue

		print("Future is Done")

def main(args=None):
	rclpy.init(args=args)
	action_client = MainActionClient()

	# test_0(action_client)
	test_1(action_client)
	# test_2(action_client)


if __name__ == '__main__':
	main()
