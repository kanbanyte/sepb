from rclpy.node import Node
from camera.camera_capture import read_crop_box, get_rgb_cropped_image, LogicalLens, open_camera
from models.object_detection_model import ObjectDetectionModel
from data_processing.case_position import convert_case_bounding_boxes
from data_processing.tray_position import determine_move
from data_processing.chip_position import get_chip_slot_number
from util.file_reader import read_yaml
from pick_place_interfaces.srv import PickPlaceService
import os
from datetime import datetime

class CameraServer(Node):
	def __init__(self):
		super().__init__('camera_server')

		# Create ROS2 services for case, chip, and tray position detection
		self.case_srv = self.create_service(PickPlaceService, 'case', self.get_case_position)
		self.chip_srv = self.create_service(PickPlaceService, 'chip', self.get_chip_position)
		self.tray_srv = self.create_service(PickPlaceService, 'tray', self.get_tray_movement)

		# Define the path to the configuration file
		# Change this path to point to your preferred configuration YAML file
		config_file_path = "/home/cobot/Documents/models/config.yaml"

		# Define the path to the output folder
		# Change this path to point to your preferred output folder
		self.__init_output_dirs( "/home/cobot/Documents/detections_outputs")

		self.get_logger().info(f"Reading configuration file '{config_file_path}'")
		self.config = read_yaml(config_file_path)

		self.__init_camera()
		self.__init_models()

	def __init_models(self):
		self.get_logger().info(f"Loading models")
		self.case_model = ObjectDetectionModel(self.config.get('model').get('detect_case'))
		self.chip_model = ObjectDetectionModel(self.config.get('model').get('detect_chip'))
		self.tray_model = ObjectDetectionModel(self.config.get('model').get('detect_tray'))

	def __init_camera(self):
		self.get_logger().info(f"Initializing the camera")
		self.camera = open_camera(self.config.get('camera'))

	def __init_output_dirs(self, output_dir):
		chip_detections_dir = os.path.join(output_dir, "chip")

		self.left_chip_detections_dir = os.path.join(chip_detections_dir, "left")
		if not os.path.exists(self.left_chip_detections_dir):
			self.get_logger().info(f"Creating output directory for chip detections from logical left lens")
			os.makedirs(self.left_chip_detections_dir)

		self.right_chip_detections_dir = os.path.join(chip_detections_dir, "right")
		if not os.path.exists(self.right_chip_detections_dir):
			self.get_logger().info(f"Creating output directory for chip detections from logical right lens")
			os.makedirs(self.right_chip_detections_dir)

		self.tray_detections_dir = os.path.join(output_dir, "tray")
		if not os.path.exists(self.tray_detections_dir):
			self.get_logger().info(f"Creating output directory for tray detections from logical right lens")
			os.makedirs(self.tray_detections_dir)

		self.case_detections_dir = os.path.join(output_dir, "case")
		if not os.path.exists(self.case_detections_dir):
			self.get_logger().info(f"Creating output directory for case detections from logical right lens")
			os.makedirs(self.case_detections_dir)

	def prepare_shutdown(self):
		# Close the camera when shutting down
		self.get_logger().info("Closing the camera")
		self.camera.close()
		self.get_logger().info("Camera closed")

	def get_case_position(self, request, response):
		# Read the crop box for case detection
		crop_box = read_crop_box(self.config.get('case_crop_box').get('right'))

		# Capture an image within the crop box from the right logical lens
		cropped_image = get_rgb_cropped_image(self.camera, crop_box, LogicalLens.RIGHT)

		# Run object detection on the cropped image
		detections = self.case_model.run_inference(
			cropped_image,
			show_image=True,
			result_img_path=self.__create_image_name(self.case_detections_dir, "case"))
		if len(detections.items()) == 0:
			# No case detected
			response.signal = -1
		else:
			# the case detection model only has 1 class ("Case")
			detected_cases = detections[0]

			# we only pick the first detected case; if the detection dictionary has a class as its key, that class is guaranteed to have at least 1 detection
			detected_case = detected_cases[0]

			# Convert bounding box to desired format
			response.signal = convert_case_bounding_boxes(detected_case)

		self.get_logger().info(f"Request: {request}, Case Number: {response.signal}")
		return response

	def __create_image_name(self, output_dir, model_name):
		current_time = datetime.now().strftime("%H-%M-%S")
		return os.path.join(output_dir, f"{model_name}.{current_time}.png")

	def get_chip_position(self, request, response):
		# Read crop boxes for left and right chip slots
		crop_boxes = self.config.get('chip_slot_crop_box')

		# get chip positions from image captured with logical left lens
		left_crop_box = read_crop_box(crop_boxes.get('left'))
		left_cropped_image = get_rgb_cropped_image(self.camera, left_crop_box, LogicalLens.LEFT)
		left_detections = self.chip_model.run_inference(
			left_cropped_image,
			show_image=True,
			result_img_path=self.__create_image_name(self.left_chip_detections_dir, "chip"))
		left_chip_positions = []
		if len(left_detections) > 0:
			left_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in left_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

		# get chip positions from image captured with logical right lens
		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(self.camera, right_crop_box, LogicalLens.RIGHT)
		right_detections = self.chip_model.run_inference(
			right_cropped_image,
			show_image=True,
			result_img_path=self.__create_image_name(self.right_chip_detections_dir, "chip"))
		right_chip_positions = []
		if len(right_detections) > 0:
			right_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in right_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

		# combine results from both lens to reduce the chance of missing a chip
		chip_positions = set(left_chip_positions + right_chip_positions)
		self.get_logger().info(f"Detected chips at the following positions: {chip_positions}")

		if len(chip_positions) == 0:
			# No chips detected
			response.signal = -1
		else:
			print(next(iter(chip_positions)))

			# Get the first detected chip position
			response.signal = next(iter(chip_positions))

		self.get_logger().info(f"Request: {request}, Chip Number: {response.signal}")
		return response

	def get_tray_movement(self, request, response):
		# Read crop boxes for tray detection
		crop_boxes = self.config.get('tray_crop_box')

		right_crop_box = read_crop_box(crop_boxes.get('right'))
		right_cropped_image = get_rgb_cropped_image(self.camera, right_crop_box, LogicalLens.RIGHT)

		# Run object detection on the cropped image and determine the best tray movement
		right_detections = self.tray_model.run_inference(
			right_cropped_image,
			show_image=True,
			result_img_path=self.__create_image_name(self.tray_detections_dir, "tray"))
		best_move = determine_move(right_detections, self.tray_model)

		response.signal = best_move.value

		self.get_logger().info(f"Request: {request}, Best Tray Movement: {best_move.name}")
		return response
