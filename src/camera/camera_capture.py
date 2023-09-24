import numpy
import pyzed.sl as sl

import sys, os; sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../"))
from data_processing import crop_image

def open_camera(camera_config):
	"""
	Open the ZED camera and applies settings specified in the yaml configuration file.

	Args:
		camera_config (dict): dictionary containing camera settings.

	Returns:
		sl.Camera: Opened and calibrated ZED camera object.
	"""
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
	if open_result != sl.ERROR_CODE.SUCCESS:
		raise ValueError(f"Failed to open camera: {open_result}")

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

def capture_image(camera):
	"""
	Captures an image using the specified camera object.

	Args:
		camera: an opened ZED camera object.

	Returns:
		np.array: The captured image as an array.
	"""
	image = sl.Mat()
	error_code = camera.grab()
	if error_code == sl.ERROR_CODE.SUCCESS:
		camera.retrieve_image(image, sl.VIEW.LEFT)
		# - get_data() turns a sl.Mat object into a numpy array
		# - accessing the image as an array without copying it will crash the program later due to unknown reasons
		image_data = numpy.copy(image.get_data())
		return image_data
	else:
		raise ValueError(f"Failed to capture image: {error_code}")

def get_rgb_cropped_image(camera, crop_box):
	"""
	Takes a photo with the camera and applies a crop box to it.

	Args:
		camera_config (dict): dictionary containing camera settings.
		crop_box (x1, y1, x2, y2): tuple containing the crop box coordinates in the (left, top, right, bottom) format.

	Returns:
		np.array: The cropped image.
	"""
	image = capture_image(camera)
	cropped_image = crop_image(image, crop_box)

	# ZED returns an image in the RGBA format but the alpha channel is not needed
	cropped_image = cropped_image[:, :, 0:3]

	# import cv2
	# from datetime import datetime
	# current_time = datetime.now().strftime("%H-%M-%S")
	# cv2.imwrite(f"raw.{current_time}.png", cropped_image)

	# TODO: does the camera needs to be closed here? (Any leaks or performance issue)
	# camera.close()
	return cropped_image

def read_crop_box(crop_box_config):
	"""
	Get the crop box in the xyxy format as specified in the yaml configuration file.

	Args:
		crop_box_config (dict): dictionary containing crop box xyxy coordinates.

	Returns:
		int, int, int, int: Coordinates of the crop box in the (left, top, right, bottom) format
	"""
	x1 = crop_box_config.get('x1')
	x2 = crop_box_config.get('x2')
	y1 = crop_box_config.get('y1')
	y2 = crop_box_config.get('y2')
	return x1, y1, x2, y2
