# Import necessary modules from ROS 2 libraries.
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
	def send_goal(self, perform_pick_place):
		# Create a PickPlaceAction action goal message.
		goal_msg = PickPlaceAction.Goal()

		# Set the goal parameter.
		goal_msg.perform_pick_place = perform_pick_place

		# Wait for the action server to be available.
		self._action_client.wait_for_server()

		# Send the goal asynchronously and return the Future object.
		return self._action_client.send_goal_async(goal_msg)
