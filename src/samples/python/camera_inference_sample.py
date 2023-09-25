import os
import time
from datetime import datetime

from data_processing.convert_case import convert_case_bounding_boxes
from util.file_dialog import select_file_from_dialog, select_folder_from_dialog
from util.file_reader import read_yaml
from models.python.object_detection_model import ObjectDetectionModel
from camera.camera_capture import read_crop_box, open_camera, get_rgb_cropped_image

def run_inference(detection_model, cropped_image, output_folder = None):
	if output_folder:
		current_time = datetime.now().strftime("%H-%M-%S")
		output_folder = os.path.join(output_folder, f"{current_time}.png")

	start_time = time.perf_counter()
	detections = detection_model.run_inference(cropped_image, output_folder)
	inference_time = time.perf_counter()

	for class_index, detected_objects in detections.items():
		for i, detected_object in enumerate(detected_objects):
			print(f"Object {i + 1}/{len(detected_objects)} in class {detection_model.classes[class_index]}: ")
			print(f"\tConfidence: {detected_object.confidence}")
			print(f"\tBox: {detected_object.bounding_box}")

	print(f"\Inference time: {inference_time - start_time:.4f} seconds")
	return detections

def main():
	file_path = select_file_from_dialog("Select YAML configuration file", ["yaml"])
	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")
		if not output_path:
			raise ValueError("Selected output path is empty")

	config = read_yaml(file_path)
	camera = open_camera(config.get('camera'))

	model = None
	cropped_image = None

	while True:
		print(
'''
===============================================
Select a model to run:
\t- 0: run chip detection model using left lens
\t- 1: run tray detection model using left lens
\t- 2: run case detection model using left lens
''')
		choice = input("Choose model to run. Press `q` to quit: ")
		if choice == 'q':
			print("Closing camera")
			camera.close()
			break
		try:
			if choice == '0':
				crop_box = read_crop_box(config.get('chip_slot_crop_box').get('left'))
				model = ObjectDetectionModel(config.get('model').get('detect_chip'))
				cropped_image = get_rgb_cropped_image(camera, crop_box)
			elif choice == '1':
				crop_box = read_crop_box(config.get('tray_crop_box').get('left'))
				model = ObjectDetectionModel(config.get('model').get('detect_tray'))
				cropped_image = get_rgb_cropped_image(camera, crop_box)
			elif choice == '2':
				crop_box = read_crop_box(config.get('case_crop_box').get('left'))
				model = ObjectDetectionModel(config.get('model').get('detect_case'))
				detections = cropped_image = get_rgb_cropped_image(camera, crop_box)
				if len(detections.items() == 0) or len(detections[0]) == 0:
					continue

				cases_bounding_boxes = detections[0]
				print(f"Case position is {convert_case_bounding_boxes(cases_bounding_boxes)}")

			else:
				raise ValueError("Invalid input")

			run_inference(model, cropped_image, output_path)
		except Exception as error:
			print(f"Error: {error}")


if __name__ == "__main__":
	main()
