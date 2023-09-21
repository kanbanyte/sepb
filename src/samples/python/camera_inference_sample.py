import time
from datetime import datetime

import sys, os; sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../"))
from util import select_file_from_dialog, select_folder_from_dialog, read_yaml
from models.python import ObjectDetectionModel
from camera import read_crop_box, open_camera, get_rgb_cropped_image

def run_inference(camera, detection_model, crop_box, output_folder = None):
	start_time = time.perf_counter()
	cropped_image = get_rgb_cropped_image(camera, crop_box)
	image_capture_time = time.perf_counter()

	if output_folder:
		current_time = datetime.now().strftime("%H-%M-%S")
		output_folder = os.path.join(output_folder, f"{current_time}.png")

	detections = detection_model.run_inference(cropped_image, output_folder)
	inference_time = time.perf_counter()

	for class_index, detected_objects in detections.items():
		for i, detected_object in enumerate(detected_objects):
			print(f"Object {i + 1}/{len(detected_objects)} in class {detection_model.classes[class_index]}: ")
			print(f"\tConfidence: {detected_object.confidence}")
			print(f"\tBox: {detected_object.bounding_box}")

	end_time = time.perf_counter()
	elapsed_time = end_time - start_time
	print(f"Total elapsed time: {elapsed_time:.4f} seconds")
	print(f"\tImage capture and crop time: {image_capture_time - start_time:.4f} seconds")
	print(f"\tInference time: {inference_time - image_capture_time:.4f} seconds")

def main():
	file_path = select_file_from_dialog("Select YAML configuration file", ["yaml"])
	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if  save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")

	config = read_yaml(file_path)
	camera = open_camera(config.get('camera'))

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
				run_inference(camera, model, crop_box, output_path)
			elif choice == '1':
				crop_box = read_crop_box(config.get('tray_crop_box').get('left'))
				model = ObjectDetectionModel(config.get('model').get('detect_tray'))
				run_inference(camera, model, crop_box, output_path)
			elif choice == '2':
				crop_box = read_crop_box(config.get('case_crop_box').get('left'))
				model = ObjectDetectionModel(config.get('model').get('detect_case'))
				run_inference(camera, model, crop_box, output_path)
			else:
				print("Invalid input")
		except Exception as error:
			print(error)


if __name__ == "__main__":
	main()
