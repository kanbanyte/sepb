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

def main():
	current_directory = os.path.dirname(os.path.realpath(__file__))
	file_name = "camera_config.yaml"
	file_path = os.path.join(current_directory, file_name)

	config = read_yaml(file_path)
	crop_box = read_crop_box(config.get('chip_slot_crop_box').get('left'))
	camera = open_camera(config.get('camera'))
	chip_detection_model = ObjectDetectionModel(config.get('model').get('detect_chip'))
	start_time = time.perf_counter()
	cropped_image = get_rgb_cropped_image(camera, crop_box)
	image_capture_time = time.perf_counter()
	bounding_boxes = chip_detection_model.run_inference(cropped_image)
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
