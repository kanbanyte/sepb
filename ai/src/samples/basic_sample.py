import cv2
import os
from datetime import datetime
from ultralytics import YOLO

from util.file_dialog import select_file_from_dialog, select_folder_from_dialog
from util.file_reader import read_yaml
from models.object_detection_model import ObjectDetectionModel

# Constants for drawing bounding boxes and text on images
BOX_THICKNESS = 2
GREEN_RGB = (0, 255, 0)
FONT_THICKNESS = 2

def select_image():
	image_file = select_file_from_dialog("Select image file", ["png", "jpg", "jpeg"])
	if not image_file:
		raise ValueError("No image file selected")

	return cv2.imread(image_file)

def run_with_ultralytics_api():
	while True:
		try:
			if input("Press 'q' to quit. Press any other key to select image and model: ") == 'q':
				print("Exiting...")
				break

			image = select_image()
			model_file = select_file_from_dialog("Select model file", ["pt"])
			if not model_file:
				raise ValueError("No model file selected")

			model = YOLO(model_file)
			results = model.predict(image, verbose=True)
			for result in results:
				cv2.imshow(f"Close window to continue", result.plot())
				cv2.waitKey(0)

			# Close all open windows when processing is complete
			cv2.destroyAllWindows()
		except Exception as error:
			print(f"Error: {error}")


def run_with_custom_api():
	config_file = select_file_from_dialog("Select configuration file", ["yaml"])
	if not config_file:
		raise ValueError("No config file selected")

	config = read_yaml(config_file)
	print("Initializing models")
	chip_model = ObjectDetectionModel(config.get('model').get('detect_chip'))
	tray_model = ObjectDetectionModel(config.get('model').get('detect_tray'))
	case_model = ObjectDetectionModel(config.get('model').get('detect_case'))

	save_output_choice = input("Save output image (y/any key)?: ")
	output_path = None
	if save_output_choice == 'y':
		output_path = select_folder_from_dialog("Select output image folder")
		if not output_path:
			raise ValueError("Error: image output folder path not selected")

	while True:
		try:
			if input("Press 'q' to quit. Press any other key to select image and model: ") == 'q':
				print("Exiting...")
				break

			image = select_image()
			print("\nChoose detection model: 0 - Chip, 1 - Case, 2 - Tray")
			model = None
			model_choice = input("Enter your choice: ")
			if model_choice == '0':
				model = chip_model
			elif model_choice == '1':
				model = case_model
			elif model_choice == '2':
				model = tray_model
			elif model_choice == 'q':
				print("Exiting...")
				break
			else:
				raise ValueError("Invalid model choice")

			output_image_path = None
			if output_path:
				print(f"Saving output images to folder '{output_path}'")
				now = datetime.utcnow().strftime("%Y-%m-%dT%H-%M-%S")
				output_image_path = os.path.join(output_path, f"{now}.png")

			print("\n===")
			detections = model.run_inference(image, result_img_path=output_image_path, show_image=True)
			print(f"Detected {len(detections)} class(es) in image")
			for class_index, detected_objects in detections.items():
				for i, detected_object in enumerate(detected_objects):
					print(f"---")
					print(f"Object {i + 1}/{len(detected_objects)} in class {model.classes[class_index]}: ")
					print(f"Confidence: {detected_object.confidence}")
					print(f"Box: {detected_object.bounding_box}")
		except Exception as error:
			print(f"Error: {error}")

def main():
	print(
'''\n
Select option to run.
- 0: Model file selected by user. Model detection configuration or image cropping supported.
- 1: Model file and detection configuration defined by user. Image cropping not supported.
''')
	option = int(input("Enter your option: "))
	if option == 0:
		run_with_ultralytics_api()
	elif option == 1:
		run_with_custom_api()

if __name__ == "__main__":
	main()
