"""
Usage:
1. Run the script using the following command:
	python yolov5_tiled_predict.py
2. Follow the on-screen prompts to select model and image files, as well as set tile dimensions.
3. The program will process the image tiles using the YOLO model and display the results.
4. Close the image window to proceed to the next tile.

Note:
- Make sure you have Python installed on your system.
- The following packages will be installed if not available: opencv-python, ultralytics
- You will be prompted to select a YOLO model file (with a .pt extension) and an image file (png, jpg, jpeg).
	If the file selection window does not appear, it might be opened in the background.
- The tile dimensions must match the dimensions used to train the model.
"""

import sys
import cv2
from ultralytics import YOLO
from src.util.file_dialog import select_file_from_dialog
from src.util.package_install import install_packages

# Constants for drawing bounding boxes and text on images
BOX_THICKNESS = 2
GREEN_RGB = (0, 255, 0)
FONT_THICKNESS = 2

# Function to split an image into tiles based on the number of rows and columns
def tile_image(image, num_rows, num_cols):
	tile_height = int(image.shape[0] / num_rows)
	tile_width = int(image.shape[1] / num_cols)
	tiled_images = []
	for row in range(num_rows):
		for col in range(num_cols):
			y_start = row * tile_height
			y_end = y_start + tile_height
			x_start = col * tile_width
			x_end = x_start + tile_width
			tile = image[y_start:y_end, x_start:x_end]
			tiled_images.append(tile)
	return tiled_images

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

# Main function that orchestrates the entire image processing workflow
def main():
	# Install required Python packages
	install_packages(["opencv-python", "ultralytics"])

	# Prompt the user to select the YOLO model file
	model_file = select_file_from_dialog("Select model file", ["pt"])
	model = YOLO(model_file)

	# Prompt the user to select the image file
	image_file = select_file_from_dialog("Select image file", ["png", "jpg", "jpeg"])
	image = cv2.imread(image_file)

	print("Select tile dimensions (must be the same as the dimension used to train the model)")
	num_rows = get_positive_int("Select row count: ", 3)
	num_cols = get_positive_int("Select column count: ", 4)

	# Split the image into tiles
	tiled_images = tile_image(image, num_rows, num_cols)

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

if __name__ == "__main__":
	main()
