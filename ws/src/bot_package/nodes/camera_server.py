from rclpy.node import Node

from camera.camera_capture import read_crop_box, get_rgb_cropped_image, LogicalLens, open_camera
from models.python.object_detection_model import ObjectDetectionModel
from data_processing.case_position import convert_case_bounding_boxes
from data_processing.tray_position import determine_move
from data_processing.chip_position import get_chip_slot_number
from util.file_reader import read_yaml

from pick_place_interfaces.srv import PickPlaceObject

class CameraServer(Node):
	def __init__(self):
		super().__init__('camera_server')
		self.case_srv = self.create_service(PickPlaceObject, 'case', self.get_case_position)
		self.chip_srv = self.create_service(PickPlaceObject, 'chip', self.get_chip_position)
		self.tray_srv = self.create_service(PickPlaceObject, 'tray', self.get_tray_movement)

		# TODO: move this path to the startup argument list
		config_file_path = "/home/cobot/Documents/models/config.yaml"
		self.get_logger().info(f"Reading configuration file '{config_file_path}'")
		self.config = read_yaml(config_file_path)

		# TODO: investigate a way to call camera.close() before destroying the node
		# right now, not calling close() has not caused any issue

		self.get_logger().info(f"Initializing the camera")
		self.camera = open_camera(self.config.get('camera'))

		self.get_logger().info(f"Loading models")
		self.case_model = ObjectDetectionModel(self.config.get('model').get('detect_case'))
		self.chip_model = ObjectDetectionModel(self.config.get('model').get('detect_chip'))
		self.tray_model = ObjectDetectionModel(self.config.get('model').get('detect_tray'))

	def prepare_shutdown(self):
		self.get_logger().info("Closing the camera")
		self.camera.close()
		self.get_logger().info("Camera closed")

	def get_case_position(self, request, response):
		crop_box = read_crop_box(self.config.get('case_crop_box').get('right'))
		cropped_image = get_rgb_cropped_image(self.camera, crop_box, LogicalLens.RIGHT)
		detections = self.case_model.run_inference(cropped_image)
		if len(detections.items()) == 0:
			response.signal = -1
		else:
			# the case detection model only has 1 class ("Case")
			detected_cases = detections[0]

			# we only pick the first detected case
			# if the detection dictionary has a class as its key, that class is guaranteed to have at least 1 detection
			detected_case = detected_cases[0]
			response.signal = convert_case_bounding_boxes(detected_case)

		self.get_logger().info(f"Request: {request}, Case Number: {response.signal}")
		return response

	def get_chip_position(self, request, response):
		crop_boxes = self.config.get('chip_slot_crop_box')

		# get chip positions from image captured with logical left lens
		left_crop_box = read_crop_box(crop_boxes.get('left'))
		left_cropped_image = get_rgb_cropped_image(self.camera, left_crop_box, LogicalLens.LEFT)
		left_detections = self.chip_model.run_inference(left_cropped_image)
		left_chip_positions = []
		if len(left_detections) > 0:
			left_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in left_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

		# get chip positions from image captured with logical right lens
		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(self.camera, right_crop_box, LogicalLens.RIGHT)
		right_detections = self.chip_model.run_inference(right_cropped_image)
		right_chip_positions = []
		if len(right_detections) > 0:
			right_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in right_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

		# combine results from both lens to reduce the chance of missing a chip
		chip_positions = set(left_chip_positions + right_chip_positions)
		self.get_logger().info(f"Detected chips at the following positions: {chip_positions}")

		if len(chip_positions) == 0:
			response.signal = -1
		else:
			print(next(iter(chip_positions)))
			response.signal = next(iter(chip_positions))

		self.get_logger().info(f"Request: {request}, Chip Number: {response.signal}")
		return response

	def get_tray_movement(self, request, response):
		crop_boxes = self.config.get('tray_crop_box')

		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(self.camera, right_crop_box, LogicalLens.RIGHT)

		right_detections = self.tray_model.run_inference(right_cropped_image)
		best_move = determine_move(right_detections, self.tray_model)

		response.signal = best_move.value

		self.get_logger().info(f"Request: {request}, Best Tray Movement: {best_move.name}")
		return response
