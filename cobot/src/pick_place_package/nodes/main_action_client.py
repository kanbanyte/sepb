# Import necessary modules from ROS 2 libraries.
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pick_place_interfaces.action import PickPlaceAction
import asyncio


# Define a class for the main action client node.
class MainActionClient(Node):
	def __init__(self):
		# Initialize the node with a name.
		super().__init__('main_action_client')

		# Create an action client to interact with the 'perform_pick_place' action server.
		self.get_logger().info("Starting action client...")
		self._action_client = ActionClient(self, PickPlaceAction, 'perform_pick_place')

	# Define a method to send a goal to the action server.
	def send_goal_async(self):
		# Create a PickPlaceAction action goal message.
		goal_msg = PickPlaceAction.Goal()
		goal_msg.goal = True

		# Wait for the action server to be available.
		self._action_client.wait_for_server()

		self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

		self._send_goal_future.add_done_callback(self.goal_response_retrieved)

		# Send the goal asynchronously and return the Future object.
		return self._send_goal_future

		# result = self._action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
		# # self._send_goal_future.add_done_callback(self.goal_response_callback)
		# if result.accepted:
		# 	self.get_logger().info('Goal accepted')
		# else:
		# 	self.get_logger().info('Goal rejected')

	def goal_response_retrieved(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected. Invalid goal.')
			return

		self.get_logger().info('Goal accepted')

		# Get goal result via an async callback
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_goal_result_async)

		# Get goal result synchronously
		# result = goal_handle.get_result()
		# get_goal_result(result)

	def get_goal_result_async(self, future):
		result = future.result().result
		self.get_logger().info(f'Task successful: {result.task_successful}')
		# should shutdown be called here?? probably dont need it since we want to do the action more than once
		# rclpy.shutdown()

	def get_goal_result(self, result):
		self.get_logger().info(f'Task successful: {result.task_successful}')
		# should shutdown be called here?? probably dont need it since we want to do the action more than once
		# rclpy.shutdown()

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(f'Received current_goal as feedback: {feedback.current_movement}')
