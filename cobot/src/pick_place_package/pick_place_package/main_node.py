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
	print(f"Done: {future.done()}")
	print(f"Result: {future.result()}")
	rclpy.spin_until_future_complete(action_client, future)

def test_2(action_client):
	# future = action_client.send_goal_async()
	# rclpy.spin_until_future_complete(action_client, future)
	print("Getting ready...")
	# time.sleep(10)
	first_time = True
	while True:
		future = action_client.send_goal_async()
		# if first_time:
		# 	time.sleep(15)
		# 	first_time = False
		ignored_responses = 0
		while not future.done() or ignored_responses != 2:

			print("Waiting for action request to complete")
			# TODO: if the loop works, but only execute once, try replacing `spin_until_future_complete`
			rclpy.spin_until_future_complete(action_client, future)
			ignored_responses = ignored_responses + 1

		break

from action_msgs.msg import GoalStatus

def test_4(action_client):
	# future = action_client.send_goal_async()
	# rclpy.spin_until_future_complete(action_client, future)
	print("Getting ready...")
	# time.sleep(10)
	first_time = True
	while True:
		future = action_client.send_goal_async()
		while True:
			print("Waiting for action request to complete")
			rclpy.spin_until_future_complete(action_client, future)

			result = future.result().result
			print(f"Result: {result}")
			if future.done() and result is not None:
				break

def test_5(action_client):
	result = action_client.send_goal_async()
	print(f"Result: {result.result}")
	print(f"Status: {result.status}")

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

# import asyncio
# async def spinning(node):
#     while rclpy.ok():
#         rclpy.spin_once(node, timeout_sec=0.01)
#         await asyncio.sleep(0.001)

# # example from ros2 github
# async def run(args, loop):

#     logger = rclpy.logging.get_logger('main_action_client')

#     # init ros2
#     rclpy.init(args=args)

#     # create node
#     action_client = MainActionClient()

#     # start spinning
#     spin_task = loop.create_task(spinning(action_client))

#     # Parallel example
#     # execute goal request and schedule in loop
#     my_task1 = loop.create_task(action_client.send_goal_async())
#     # my_task2 = loop.create_task(action_client.send_goal())

#     # glue futures together and wait
#     # wait_future = asyncio.wait([my_task1, my_task2])
#     wait_future = asyncio.wait([my_task1])

#     # run event loop
#     finished, unfinished = await wait_future
#     logger.info(f'unfinished: {len(unfinished)}')
#     for task in finished:
#         logger.info('result {} and status flag {}'.format(*task.result()))

#     # Sequence
#     result, status = await loop.create_task(action_client.send_goal_async())
#     logger.info(f'A) result {result} and status flag {status}')
#     # result, status = await loop.create_task(action_client.send_goal())
#     # logger.info(f'B) result {result} and status flag {status}')

#     # cancel spinning task
#     spin_task.cancel()
#     try:
#         await spin_task
#     except asyncio.exceptions.CancelledError:
#         pass
#     rclpy.shutdown()


# def main(args=None):
#     loop = asyncio.get_event_loop()
#     loop.run_until_complete(run(args, loop=loop))


def main(args=None):
	rclpy.init(args=args)
	action_client = MainActionClient()

	# test_0(action_client)
	# test_1(action_client)
	# test_2(action_client)
	# test_4(action_client)

	while True:
		test_5(action_client)



if __name__ == '__main__':
	main()
