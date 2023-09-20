import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../util"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../data_processing"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../models/python"))
from datetime import datetime

import cv2
from ultralytics import YOLO
from file_dialog import select_file_from_dialog, select_folder_from_dialog
from file_reader import read_yaml
from image_processing import tile_image
from object_detection_model import ObjectDetectionModel

# Constants for drawing bounding boxes and text on images
BOX_THICKNESS = 2
GREEN_RGB = (0, 255, 0)
FONT_THICKNESS = 2

# Function to get a positive integer input from the user
def get_positive_int(prompt, default_value):
	try:
		user_input = input(prompt)
		if (not user_input):
			print(f"No value entered, using default value: {default_value}")
			return default_value

		value = int(user_input)
		if value <= 0:
			print("Input must be a positive integer.")
			sys.exit()
		else:
			return value
	except ValueError:
		print("Input must be a positive integer.")
		sys.exit()

def version_0(tiled_images):
	# Prompt the user to select the YOLO model file
	model_file = select_file_from_dialog("Select model file", ["pt"])
	if model_file is None:
		print("No model file selected")
		exit(-1)
	model = YOLO(model_file)
	for tile_index, tile in enumerate(tiled_images):
		print(f"Checking tile {tile_index}")
		results = model.predict(tile, verbose=False)

		for result in results:
			# This is the built-in function that plots the bounding boxes
			# However, we deliberately draw boxes manually to make sure the data is extracted correctly
			# cv2.imshow(f"Tile (close window to continue)", result.plot())

			if result.boxes is None or result.boxes.xyxy.numel() == 0:
				print(f"No object detected in tile {tile_index}")
				cv2.imshow(f'Empty tile #{tile_index} (CLOSE WINDOW TO CONTINUE)', tile)
				cv2.waitKey(0)
				continue

			x1_tensor = result.boxes.xyxy[:, 0]
			y1_tensor = result.boxes.xyxy[:, 1]
			x2_tensor = result.boxes.xyxy[:, 2]
			y2_tensor = result.boxes.xyxy[:, 3]

			detected_obj_count = x1_tensor.size(dim=-1)
			print(f"Displaying {detected_obj_count} detected object(s). Close windows to continue")

			tile_result_data = zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor)
			for obj_index, (conf, x1, y1, x2, y2) in enumerate(tile_result_data):
				# OpenCV only works with integers
				x1_int = int(x1)
				y1_int = int(y1)
				x2_int = int(x2)
				y2_int = int(y2)

				# Draw bounding boxes and confidence scores on the tile image
				cv2.rectangle(tile, (x1_int, y1_int), (x2_int, y2_int), GREEN_RGB, BOX_THICKNESS)
				cv2.putText(tile, f'{conf * 100:.2f}', (x1_int, y1_int - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN_RGB, FONT_THICKNESS)

				# Display object information and performance statistics
				print(f"Object #{obj_index}/{detected_obj_count} - Tile #{tile_index}")
				print("Performance in ms:")
				for key, value in result.speed.items():
					print(f"\t{key}: {value}")
				print(f"Confidence: {conf}")
				print(
					f"Bounding Box: "
					f"(x1={x1:.2f}, y1={y1:.2f}), "
					f"(x2={x2:.2f}, y2={y2:.2f})"
				)
				print()
			# Display the tile with bounding boxes and information
			cv2.imshow(f'Tile #{tile_index} (CLOSE WINDOW TO CONTINUE)', tile)
			# Wait for user to close the tile image window
			cv2.waitKey(0)
	# Close all open windows when processing is complete
	cv2.destroyAllWindows()

def version_1(tiled_images):
	config_file = select_file_from_dialog("Select configuration file", ["yaml"])
	if config_file is None:
		print("No config file selected")
		exit(-1)
	config = read_yaml(config_file)

	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")

	if not output_path:
		print("Error: image output folder path not selected")
		exit(-1)

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

# Main function that orchestrates the entire image processing workflow
def main():
	# Prompt the user to select the image file
	image_file = select_file_from_dialog("Select image file", ["png", "jpg", "jpeg"])
	if not image_file:
		print("No image file selected")
		exit(-1)
	image = cv2.imread(image_file)

	print("Select tile dimensions (must be the same as the dimension used to train the model)")
	num_rows = get_positive_int("Select row count: ", 1)
	num_cols = get_positive_int("Select column count: ", 1)

	# Split the image into tiles
	tiled_images = tile_image(image, num_rows, num_cols)
	print(
'''
Select version to run.
- Version 0 shows individual tiles with bounding boxes and asks the user to select the model file
- Version 1 only prints bounding boxes with confidence level and asks the user to select the configuration file
''')
	version = int(input("Select version number to run: "))
	if version == 0:
		version_0(tiled_images)
	elif version == 1:
		version_1(tiled_images)

if __name__ == "__main__":
	main()
