from rclpy.node import Node

from camera.camera_capture import *
from models.python.object_detection_model import *
from models.python.detected_object import *
from data_processing.case_position import *
from data_processing.tray_position import *
from data_processing.chip_position import *

from pick_place_interfaces.srv import Case
from pick_place_interfaces.srv import Tray
from pick_place_interfaces.srv import Chip

class CameraServer(Node):
	def __init__(self):
		super().__init__('camera_server')
		self.case_srv = self.create_service(Case, 'case', self.add_two_ints_callback)
		# self.chip_srv = self.create_service(Chip, 'chip', self.add_two_ints_callback)
		# self.tray_srv = self.create_service(Tray, 'tray', self.add_two_ints_callback)

	def add_two_ints_callback(self, request, response):
		# response.sum = request.a + request.b
		# self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

		# return response
		response.case_number = 5
		self.get_logger().info(f"Request: {request}, Case Number: {response.case_number}")

		return response
