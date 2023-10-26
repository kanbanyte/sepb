import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pick_place_interfaces.action import PickPlaceAction

# Define a class for the main action client node.
class MainActionClient(Node):
	def __init__(self):
		# Initialize the node with a name.
		super().__init__('main_action_client')

		# Create an action client to interact with the 'perform_pick_place' action server.
		self.get_logger().info("Starting action client...")
		self._action_client = ActionClient(self, PickPlaceAction, 'perform_pick_place')

	def send_pick_place_request(self):
		'''
		Send a request that completes a pick and place cycle to the server.
		'''

		self.get_logger().info("Sending request to server...")

		# Create a PickPlaceAction action goal message.
		goal_msg = PickPlaceAction.Goal()
		goal_msg.goal = True

		# Wait for the action server to be available.
		self._action_client.wait_for_server()

		# Sends a goal and waits until the server processes it
		goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		rclpy.spin_until_future_complete(self, goal_future)

		# Sends a request for the result and waits until the server processes it
		result_handle = goal_future.result()
		result_future = result_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)

		self.get_logger().info("Pick and Place request completed")
		self.get_logger().info(f"Result: task_successful={result_future.result().result.task_successful}")

		# TODO: investigate how to convert this number to a string name
		self.get_logger().info(f"Status: {result_future.result().status}")

		return result_future.result()

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(f'Received current_goal as feedback: {feedback.current_movement}')
