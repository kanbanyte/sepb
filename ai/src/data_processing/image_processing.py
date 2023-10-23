import cv2
from PIL import Image
import tkinter as tk

def crop_image(image, crop_box):
	'''
	Crops an image based on the specified crop box.

	Args:
		image (np.array): input image as a NumPy array.
		crop_box (x1, y1, x2, y2): Tuple containing crop box coordinates (left, top, right, bottom).

	Returns:
		np.array: cropped image
	'''
	if image.size == 0:
		raise ValueError(f"Input image size must not be 0")

	x1, y1, x2, y2 = crop_box
	height, width, _ = image.shape
	if abs(x2 - x1) > width:
		raise ValueError("Crop box width is larger than image width")

	if abs(y2 - y1) > height:
		raise ValueError("Crop box height is larger than image height")

	x1, y1, x2, y2 = crop_box
	return image[y1:y2, x1:x2]

def draw_bounding_box(image, bounding_box, name):
	'''
	Draw a green bounding box on an image.

	Args:
		image (np.array): input image as a NumPy array.
		bounding_box (x1, y1, x2, y2): bounding box coordinates in the (left, top, right, bottom) format.
		name (str): name of the class of the bounding box.

	Returns:
		None
	'''
	x1, y1, x2, y2 = bounding_box
	image_top_left = (round(x1), round(y1))
	image_bottom_right = (round(x2), round(y2))
	green = (0, 255, 0)

	image_height, image_width, _ = image.shape

	# magic number that makes the font size legible within images of different sizes
	# this number is obtained through trial-and-error, change it to suit your preference
	font_scale = (image_width + image_height) / 1200
	font = cv2.FONT_HERSHEY_DUPLEX

	# position the text below the image but shift it up if the text goes out of view
	(_, text_height), _ = cv2.getTextSize(name, font, font_scale, thickness=1)
	text_bottom_left = (round(image_top_left[0]), round(min(image_bottom_right[1] + text_height + 2, image_height - text_height)))
	cv2.putText(
		image,
		name.upper(),
		text_bottom_left,
		font,
		font_scale,
		green,
		thickness=1,
		lineType=cv2.LINE_AA)

	cv2.rectangle(image, image_top_left, image_bottom_right, color=green, thickness=2)

def __get_screen_height():
	root = tk.Tk()
	screen_height = root.winfo_screenheight()
	root.destroy()

	return screen_height

def show_image(image, name='Image'):
	'''
	Show image without blocking the current thread.

	Args:
		image (np.array): input image as a NumPy array.
		name (str): window name.

	Returns:
		None
	'''

	# resize the image to make its height 3/4 of the screen height whilst maintaining aspect ratio
	image_pil = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	screen_height = __get_screen_height()
	new_height = screen_height * 3 / 4

	# get the width/height ratio
	aspect_ratio = float(image.shape[1]) / float(image.shape[0])
	new_dimensions = (int(aspect_ratio * new_height), int(new_height))
	Image.fromarray(image_pil).resize(new_dimensions).show(title=name)
