import rclpy
from rclpy.node import Node
from ur_dashboard_msgs.srv import Load
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger

import time

class GripperServer(Node):
	def __init__(self):
		super().__init__("gripper_server")

		# Create a service for the gripper
		self.srv = self.create_service(SetIO, 'gripper_service', self.gripper_callback)

		# Create client for /dashboard_client/load_program service and load gripper program
		self.subnode = rclpy.create_node('subnode')

		# Create clients for various services
		self.play_cli = self.subnode.create_client(Trigger, '/dashboard_client/play')
		self.load_program_cli = self.subnode.create_client(Load, '/dashboard_client/load_program')
		self.switch_pin_io_cli = self.subnode.create_client(SetIO, '/io_and_status_controller/set_io')

		# Define the gripper program and ros program file names
		self.gripper_file_name = 'gripper_test.urp'
		self.ros_file_name = 'ROS.urp'

		# Wait for services to become available
		while not self.load_program_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.load_program_cli.srv_name} service not available, trying again...")
		while not self.switch_pin_io_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.switch_pin_io_cli.srv_name} service not available, trying again...")

		self.get_logger().info(f"Started gripper service...")

	def play_program(self):
		play_request = Trigger.Request()

		# Call the play program service asynchronously
		self.future = self.play_cli.call_async(play_request)
		rclpy.spin_until_future_complete(self.subnode, self.future)

		return self.future

	def load_file(self, file_name):
		# Create a request to load a program file
		load_request = Load.Request()
		load_request.filename = file_name

		# Call the load program service asynchronously
		self.future = self.load_program_cli.call_async(load_request)
		rclpy.spin_until_future_complete(self.subnode, self.future)

		return self.future.result()

	def switch_pin_io(self, request):
		# Call the switch pin I/O service asynchronously
		self.future = self.switch_pin_io_cli.call_async(request)

		return self.future

	def load_and_play_program(self, file_name):
		self.loaded_program = self.load_file(file_name).success
		self.get_logger().info(f"Loaded program success: {self.loaded_program}")

		# Play the loaded program and log the result
		self.future_play = self.play_program()
		while not self.future_play.result().success:
			self.future_play = self.play_program()
			rclpy.spin_until_future_complete(self.subnode, self.future_play)

		self.program_playing = self.future_play.result().success

		success_string = f"Success: {self.program_playing}"
		if self.program_playing:
			self.get_logger().info(f"{success_string}. Playing program...")
		else:
			self.get_logger().info(f"{success_string}. Program failed to play...")

	def gripper_callback(self, request, response):
		# Load the gripper program and log the result
		self.load_and_play_program(self.gripper_file_name)

		self.get_logger().info(f"Receiving request: pin={request.pin}, state={request.state}")

		# Create a request to switch pin I/O
		switch_io_request = SetIO.Request()
		switch_io_request.fun = 1

		switch_io_request.pin = request.pin
		switch_io_request.state = request.state

		# Call the switch pin I/O service asynchronously
		future = self.switch_pin_io(switch_io_request)
		rclpy.spin_until_future_complete(self.subnode, future)

		if future.result() is not None:
			result = future.result()
			self.get_logger().info(f"Gripper service call success: {result.success}")
			response.success = result.success

			time.sleep(1)

			# Load the gripper program and log the result
			self.load_and_play_program(self.ros_file_name)

			return response
		else:
			self.get_logger().error(f"Error when calling service: {future.exception()}")
