from rclpy.node import Node

# TODO: fix import to avoid * imports
from camera.camera_capture import *
from models.python.object_detection_model import *
from models.python.detected_object import *
from data_processing.case_position import *
from data_processing.tray_position import *
from data_processing.chip_position import *
from util.file_reader import *

from pick_place_interfaces.srv import Case
from pick_place_interfaces.srv import Tray
from pick_place_interfaces.srv import Chip

class CameraServer(Node):
	def __init__(self):
		super().__init__('camera_server')
		self.case_srv = self.create_service(Case, 'case', self.get_case_position)
		self.chip_srv = self.create_service(Chip, 'chip', self.get_chip_position)
		# self.tray_srv = self.create_service(Tray, 'tray', self.add_two_ints_callback)

	def get_case_position(self, request, response):
		# response.sum = request.a + request.b
		# self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

		# TODO: move this path to the startup argument list
		config_file_path = "/home/cobot/Documents/models/config.yaml"
		config = read_yaml(config_file_path)
		crop_box = read_crop_box(config.get('case_crop_box').get('right'))
		model = ObjectDetectionModel(config.get('model').get('detect_case'))

		camera = open_camera(config.get('camera'))
		cropped_image = get_rgb_cropped_image(camera, crop_box, LogicalLens.RIGHT)
		detections = model.run_inference(cropped_image)

		if len(detections.items()) == 0:
			response.case_number = -1
		else:
			# the case detection model only has 1 class ("Case")
			detected_cases = detections[0]

			# we only pick the first detected case
			# if the detection dictionary has a class as its key, that class is guaranteed to have at least 1 detection
			detected_case = detected_cases[0]
			response.case_number = convert_case_bounding_boxes(detected_case)

		self.get_logger().info(f"Request: {request}, Case Number: {response.case_number}")
		return response

	def get_chip_position(self, request, response):
		# TODO: move this path to the startup argument list
		config_file_path = "/home/cobot/Documents/models/config.yaml"
		config = read_yaml(config_file_path)
		crop_boxes = config.get('chip')
		camera = open_camera(config.get('camera'))
		model = ObjectDetectionModel(config.get('model').get('detect_case'))

		left_crop_box = read_crop_box(crop_boxes.get('left'))
		left_cropped_image = get_rgb_cropped_image(camera, left_crop_box, LogicalLens.LEFT)
		left_detections = model.run_inference(left_cropped_image)
		left_chip_positions = get_chip_slot_number(left_chip_positions[0]) if len(left_detections) > 0 else []

		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(camera, right_crop_box, LogicalLens.RIGHT)
		right_detections = model.run_inference(right_cropped_image)
		right_chip_positions = get_chip_slot_number(left_chip_positions[0]) if len(right_detections) > 0 else []

		chip_positions =  set(left_chip_positions + right_chip_positions)
		self.get_logger().info(f"Detected chips at the following positions: {chip_positions}")

		if len(chip_positions) == 0:
			response.case_number = -1
		else:
			response.case_number = convert_case_bounding_boxes(chip_positions[0])

		self.get_logger().info(f"Request: {request}, Chip Number: {response.case_number}")
		return response

	def get_tray_movement(self, request, response):
		config_file_path = "/home/cobot/Documents/models/config.yaml"
		config = read_yaml(config_file_path)
		crop_boxes = config.get('tray')
		camera = open_camera(config.get('camera'))
		model = ObjectDetectionModel(config.get('model').get('detect_case'))

		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(camera, right_crop_box, LogicalLens.RIGHT)
		right_detections = model.run_inference(right_cropped_image)

		best_move = determine_move(right_detections, model)
		response.tray_movement = int(best_move)

		self.get_logger().info(f"Request: {request}, Best Tray Movement: {best_move.name}")
		return response
