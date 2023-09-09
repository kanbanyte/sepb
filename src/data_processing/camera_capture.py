import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
from cli_runner import install_packages
install_packages(["opencv-python", "cython", "numpy", "pyopengl", "yaml", "ultralytics"])

import cv2
import time
from datetime import datetime
from ultralytics import YOLO
import pyzed.sl as sl

from file_reader import read_yaml
from file_dialog import select_file_from_dialog

def open_camera(camera_config):
	brightness = camera_config.get('brightness')
	contrast = camera_config.get('contrast')
	hue = camera_config.get('hue')
	saturation = camera_config.get('saturation')
	sharpness = camera_config.get('sharpness')
	gamma = camera_config.get('gamma')
	white_balance = camera_config.get('white_balance')
	gain = camera_config.get('gain')
	exposure = camera_config.get('exposure')

	init_params = sl.InitParameters()
	init_params.camera_resolution = sl.RESOLUTION.HD2K
	init_params.camera_image_flip = sl.FLIP_MODE.ON
	camera = sl.Camera()
	open_result = camera.open(init_params)
	if  open_result != sl.ERROR_CODE.SUCCESS:
		print(f"Failed to open camera: {open_result}")
		exit(-1)

	camera.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, contrast)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, saturation)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, brightness)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.HUE, hue)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, sharpness)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, gamma)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, exposure)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, gain)
	camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, white_balance)

	return camera

def get_rgb_cropped_image(camera, crop_box):
	x1, x2, y1, y2 = crop_box
	image = sl.Mat()
	err = camera.grab()
	if err == sl.ERROR_CODE.SUCCESS:
		camera.retrieve_image(image, sl.VIEW.LEFT)
		current_time = datetime.now().strftime("%H-%M-%S")
		cv2.imwrite(f"{current_time}.png", image.get_data())
		cropped_image = apply_crop(x1, x2, y1, y2, image)
		camera.close()
		cropped_image = cropped_image[:, :, 0:3]
		return cropped_image
	else:
		print(f"Error: {err}")
		exit(-1)

def apply_crop(x1, y1, x2, y2, image):
	cropped_image = image.get_data()[y1:y2, x1:x2]
	current_time = datetime.now().strftime("%H-%M-%S")
	cv2.imwrite(f"cropped.{current_time}.png", cropped_image)
	return cropped_image

def read_crop_box(crop_box_config):
	x1 = crop_box_config.get('x1')
	x2 = crop_box_config.get('y1')
	y1 = crop_box_config.get('x2')
	y2 = crop_box_config.get('y2')
	return x1, x2, y1, y2

def get_bounding_boxes(image, model):
	results = model.predict(image, verbose=False)
	bounding_boxes = []
	for result in results:
		if result.boxes is None or result.boxes.xyxy.numel() == 0:
			continue

		x1_tensor = result.boxes.xyxy[:, 0]
		y1_tensor = result.boxes.xyxy[:, 1]
		x2_tensor = result.boxes.xyxy[:, 2]
		y2_tensor = result.boxes.xyxy[:, 3]

		tile_result_data = zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor)
		for (conf, x1, y1, x2, y2) in tile_result_data:
			bounding_boxes.append(conf, x1, y1, x2, y2)

	return bounding_boxes

def load_model(model_file):
	# Prompt the user to select the YOLO model file
	# model_file = select_file_from_dialog("Select model file", ["pt"])
	# if model_file is None:
	# 	print("No model file selected")
	# 	exit -1

	model = YOLO(model_file)
	return model

def main():
	current_directory = os.path.dirname(os.path.realpath(__file__))
	file_name = "camera_config.yaml"
	file_path = os.path.join(current_directory, file_name)
	config = read_yaml(file_path)
	crop_box = read_crop_box(config.get('chip_slot_crop_box').get('left'))
	camera = open_camera(config.get('camera'))

	model = load_model(config.get('model'))

	start_time = time.perf_counter()
	cropped_image = get_rgb_cropped_image(camera, crop_box)
	image_capture_time = time.perf_counter()
	bounding_boxes = get_bounding_boxes(cropped_image, model)
	inference_time = time.perf_counter()
	for i, bounding_box in enumerate(bounding_boxes):
		print(f"Bounding box #{i}")
		print(f"(confidence, x1, y1, x2, y2): {bounding_box}")

	end_time = time.perf_counter()
	elapsed_time = end_time - start_time
	print(f"Total elapsed time: {elapsed_time:.4f} seconds")
	print(f"\tImage capture and crop time: {image_capture_time - start_time:.4f} seconds")
	print(f"\tInference time: {inference_time - image_capture_time:.4f} seconds")



if __name__ == "__main__":
	main()
