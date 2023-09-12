import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../util"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../models/python"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../data_processing"))
from cli_runner import install_packages
install_packages(["opencv-python", "cython", "numpy", "pyopengl", "yaml", "ultralytics"])

import time
from file_reader import read_yaml
from object_detection_model import ObjectDetectionModel
from camera_capture import read_crop_box, open_camera, get_rgb_cropped_image
from file_dialog import select_file_from_dialog

def run_inference(camera, detection_model, crop_box):
	start_time = time.perf_counter()
	cropped_image = get_rgb_cropped_image(camera, crop_box)
	image_capture_time = time.perf_counter()
	bounding_boxes = detection_model.run_inference(cropped_image)
	inference_time = time.perf_counter()
	for i, bounding_box in enumerate(bounding_boxes):
		print(f"Bounding box #{i}")
		print(f"(confidence, x1, y1, x2, y2): {bounding_box}")

	end_time = time.perf_counter()
	elapsed_time = end_time - start_time
	print(f"Total elapsed time: {elapsed_time:.4f} seconds")
	print(f"\tImage capture and crop time: {image_capture_time - start_time:.4f} seconds")
	print(f"\tInference time: {inference_time - image_capture_time:.4f} seconds")

def main():
	file_path = select_file_from_dialog("Select YAML configuration file")

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
		choice = input("Choose model to run. Press `q` to quit")
		if choice == 'q':
			camera.close()
			break

		if input == 0:
			crop_box = read_crop_box(config.get('chip_slot_crop_box').get('left'))
			model = ObjectDetectionModel(config.get('model').get('detect_chip'))
			run_inference(camera, model, crop_box)
		elif input == 1:
			crop_box = read_crop_box(config.get('tray_crop_box').get('left'))
			model = ObjectDetectionModel(config.get('model').get('detect_tray'))
			run_inference(camera, model, crop_box)
		elif input == 2:
			crop_box = read_crop_box(config.get('case_crop_box').get('left'))
			model = ObjectDetectionModel(config.get('model').get('detect_case'))
			run_inference(camera, model, crop_box)

if __name__ == "__main__":
	main()
