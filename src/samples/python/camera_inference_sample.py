import os
import time
from datetime import datetime
from camera.camera_lens import LogicalLens

from data_processing.image_processing import draw_bounding_box
from data_processing.case_position import convert_case_bounding_boxes
from data_processing.tray_position import determine_move
from data_processing.chip_position import get_chip_slot_number
from util.file_dialog import select_file_from_dialog, select_folder_from_dialog
from util.file_reader import read_yaml
from models.python.object_detection_model import ObjectDetectionModel
from camera.camera_capture import read_crop_box, open_camera, get_rgb_cropped_image

def run_inference(detection_model, cropped_image, output_location = None):
	if output_location:
		current_time = datetime.now().strftime("%H-%M-%S")
		output_location = os.path.join(output_location, f"{current_time}.png")

	start_time = time.perf_counter()
	detections = detection_model.run_inference(cropped_image, output_location, show_image=True)
	inference_time = time.perf_counter()

	for class_index, detected_objects in detections.items():
		for i, detected_tray in enumerate(detected_objects):
			print(f"Object {i + 1}/{len(detected_objects)} in class {detection_model.classes[class_index]}: ")
			print(f"\tConfidence: {detected_tray.confidence}")
			print(f"\tBox: {detected_tray.bounding_box}")

	print(f"Inference time: {inference_time - start_time:.4f} seconds")
	return detections

def main():
	file_path = select_file_from_dialog("Select YAML configuration file", ["yaml"])
	if not file_path:
		raise ValueError("No configuration file selected")

	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")
		if not output_path:
			raise ValueError("Selected output path is empty")

		if not os.path.exists(output_path):
			os.mkdir(output_path)

	config = read_yaml(file_path)
	camera = open_camera(config.get('camera'))

	model = None
	cropped_image = None

	while True:
		print(
'''
===============================================
Select a model to run:
\t- 0: run chip detection model using both lenses
\t- 1: run tray detection model using physical left lens
\t- 2: run case detection model using physical left lens
''')
		choice = input("Choose model to run. Press `q` to quit: ")
		if choice == 'q':
			print("Closing camera")
			camera.close()
			break

		try:
			if choice == '0':
				crop_boxes = config.get('chip_slot_crop_box')
				model = ObjectDetectionModel(config.get('model').get('detect_chip'))

				left_lens_output_dir = None
				right_lens_output_dir = None

				if output_path:
					# run inference using the logical left lens and calculate the chips' positions
					left_lens_output_dir = os.path.join(output_path, "left")
					if not os.path.exists(left_lens_output_dir):
						os.mkdir(left_lens_output_dir)
					print(f"Saving output of left lens in '{left_lens_output_dir}'")
		
					right_lens_output_dir = os.path.join(output_path, "right")
					if not os.path.exists(right_lens_output_dir):
						os.mkdir(right_lens_output_dir)
					print(f"Saving output of right lens in '{right_lens_output_dir}'")

				# get chip positions from image captured with logical left lens
				left_crop_box = read_crop_box(crop_boxes.get('left'))
				left_cropped_image = get_rgb_cropped_image(camera, left_crop_box, LogicalLens.LEFT)
				left_detections = run_inference(model, left_cropped_image, left_lens_output_dir)
				left_chip_positions = []
				if len(left_detections) > 0:
					left_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in left_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

				# get chip positions from image captured with logical right lens
				right_crop_box = read_crop_box(crop_boxes.get('right'))
				right_cropped_image = get_rgb_cropped_image(camera, right_crop_box, LogicalLens.RIGHT)
				right_detections = run_inference(model, right_cropped_image, left_lens_output_dir)
				right_chip_positions = []
				if len(right_detections) > 0:
					right_chip_positions.extend(get_chip_slot_number(x.bounding_box) for x in right_detections[0] if get_chip_slot_number(x.bounding_box) is not None)

				# combine results from both lens to reduce the chance of missing a chip
				chip_positions = set(left_chip_positions + right_chip_positions)

				chip_number = -1
				if len(chip_positions) > 0:
					chip_number = next(iter(chip_positions))

				print(f"Detected chip positions: {chip_positions}")
				print(f"Chip selected for transfer: {chip_number}")

			elif choice == '1':
				crop_box = read_crop_box(config.get('tray_crop_box').get('right'))
				model = ObjectDetectionModel(config.get('model').get('detect_tray'))
				cropped_image = get_rgb_cropped_image(camera, crop_box, LogicalLens.RIGHT)
				detections = run_inference(model, cropped_image, output_path)
				best_move = determine_move(detections, model)

				print(f"Best Tray movement is: {best_move.name}")

			elif choice == '2':
				crop_box = read_crop_box(config.get('case_crop_box').get('right'))
				model = ObjectDetectionModel(config.get('model').get('detect_case'))
				cropped_image = get_rgb_cropped_image(camera, crop_box, LogicalLens.RIGHT)
				detections = run_inference(model, cropped_image, output_path)

				if len(detections.items()) == 0:
					continue

				# there is only one class which is the case and its index is 0
				for i, detected_case in enumerate(detections[0]):
					print(f"Detected case #{i} position: {convert_case_bounding_boxes(detected_case)}")

			else:
				raise ValueError("Invalid input")
		except Exception as error:
			print(f"Error: {error}")


if __name__ == "__main__":
	main()
