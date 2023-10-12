# Import necessary modules from ROS 2 libraries.
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

	# Define a method to send a goal to the action server.
	def send_goal(self, goal):
		# Create a PickPlaceAction action goal message.
		goal_msg = PickPlaceAction.Goal()
		goal_msg.goal = goal

		# Wait for the action server to be available.
		self._action_client.wait_for_server()

		self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self._send_goal_future.add_done_callback(self.goal_response_callback)

		# Send the goal asynchronously and return the Future object.
		return self._action_client.send_goal_async(goal_msg)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected. Invalid goal.')
			return

		self.get_logger().info('Goal accepted')

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info(f'Task successful: {result.task_successful}')
		rclpy.shutdown()

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(f'Received current_goal as feedback: {feedback.current_movement}')
