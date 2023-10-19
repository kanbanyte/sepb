# Import the rclpy library for working with ROS 2 Python.
import rclpy
# Import the MainActionClient class from the 'main_action_client' module.
from nodes.main_action_client import MainActionClient


def do_pick_place_async(action_client):
	while True:
		future = action_client.send_goal(True)
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

def main(args=None):
	# Initialize the ROS 2 Python node.
	rclpy.init(args=args)

	# Create an instance of the MainActionClient class, which is responsible for sending action goals.
	action_client = MainActionClient()
	global Flag
	# do_pick_place_async(action_client)
	while True:
		with FlagLock:
			print("Flag reset")
			Flag = False
		# lets test working working working  LETS wait for a few loops, are they working?, like are the loops running as expected???
		# it went through the pick place task but now it's stopped only did it once
		future = action_client.send_goal(True)
		# the callback is not called goal_response_callback?
		# no, set_done_flag
		# wait, nvm
		future.add_done_callback(set_done_flag)
		while True:
			FlagLock.acquire(blocking=True, timeout=2)
			if Flag is True:
				FlagLock.release()
				break
			else:
				FlagLock.release()
				# continue looping here
				print("Sleeping for 2 secs")
				time.sleep(2)

				rclpy.spin_until_future_complete(action_client, future)
				continue

		print("Future is Done")
# lets try again
# the stupid callback is not called

if __name__ == '__main__':
	# Call the main function when this script is executed directly.
	main()

	# did you see the Goal Accepeted line get printed out? Seems weird, See? Kinda strange that the future is done immediately
