import rclpy
from rclpy.node import Node
# from pick_place_interfaces.srv import URLoad
from ur_dashboard_msgs.srv import Load


class GripperServer(Node):
	def __init__(self):
		super().__init__("gripper_server")

		# Create client for /dashboard_client/load_program service and load gripper program
		self.load_program_cli = self.create_client(Load, '/dashboard_client/load_program')

		load_msg = Load.Request()
		load_msg.filename = 'gripper_test.urp'

		while not self.load_program_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.load_program_cli.srv_name} service not available, trying again...")

		self.loaded_program = self.load_gripper_file(load_msg.filename).success
		self.get_logger().info(f"Loaded program success: {self.loaded_program}")

	def load_gripper_file(self, file_name):
		load_request = Load.Request()
		load_request.filename = file_name
		# self.future = self.load_program_cli.call_async(load_request)
		self.future = self.load_program_cli.call(load_request)
		rclpy.spin_until_future_complete(self, self.future)

		return self.future.result()
