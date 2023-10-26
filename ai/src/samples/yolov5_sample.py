import cv2
import os
from datetime import datetime
from ultralytics import YOLO

from util.file_dialog import select_file_from_dialog, select_folder_from_dialog
from util.file_reader import read_yaml
from data_processing.image_processing import tile_image
from models.object_detection_model import ObjectDetectionModel

# Constants for drawing bounding boxes and text on images
BOX_THICKNESS = 2
GREEN_RGB = (0, 255, 0)
FONT_THICKNESS = 2

def version_0(tiled_images):
	model_file = select_file_from_dialog("Select model file", ["pt"])
	if not model_file:
		raise ValueError("No model file selected")

	model = YOLO(model_file)
	for tile_index, tile in enumerate(tiled_images):
		results = model.predict(tile, verbose=True)
		for result in results:
			cv2.imshow(f"Tile {tile_index + 1}/{len(tiled_images)} (close window to continue)", result.plot())
			cv2.waitKey(0)

	# Close all open windows when processing is complete
	cv2.destroyAllWindows()

def version_1(tiled_images):
	config_file = select_file_from_dialog("Select configuration file", ["yaml"])
	if not config_file:
		raise ValueError("No config file selected")

	config = read_yaml(config_file)

	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")
		if not output_path:
			raise ValueError("Error: image output folder path not selected")

	print(f"Saving output images to folder '{output_path}'")
	now = datetime.utcnow().strftime("%Y-%m-%dT%H-%M-%S")
	model = ObjectDetectionModel(config.get('model').get('detect_chip'))
	for tile_index, tile in enumerate(tiled_images):
		output_image_path = os.path.join(output_path, f"{now}-tile-{tile_index}.png")
		print(f"Checking tile {tile_index}")
		detections = model.run_inference(tile, output_image_path)
		print("======")
		print(f"Detected {len(detections)} class(es) in tile {tile_index}")
		for class_index, detected_objects in detections.items():
			for i, detected_object in enumerate(detected_objects):
				print(f"Object {i + 1}/{len(detected_objects)} in class {model.classes[class_index]}: ")
				print(f"\tConfidence: {detected_object.confidence}")
				print(f"\tBox: {detected_object.bounding_box}")
		print()

def main():
	image_file = select_file_from_dialog("Select image file", ["png", "jpg", "jpeg"])
	if not image_file:
		raise ValueError("No image file selected")

	image = cv2.imread(image_file)

	print("Select tile dimensions (must be the same as the dimension used to train the model)")
	num_rows = int(input("Select row count: "))
	num_cols = int(input("Select column count: "))

	# Split the image into tiles
	tiled_images = tile_image(image, num_rows, num_cols)
	print(
'''
Select option to run.
- 0: shows individual image tiles with bounding boxes and print result details to the console
- 1: only prints bounding boxes with confidence level and asks the user to select the configuration file
''')
	version = int(input("Enter your option: "))
	if version == 0:
		version_0(tiled_images)
	elif version == 1:
		version_1(tiled_images)

if __name__ == "__main__":
	main()