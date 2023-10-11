import cv2
import threading
from threading import Lock
import tkinter as tk

def crop_image(image, crop_box):
	"""
	Crops an image based on the specified crop box.

	Args:
		image (np.array): Input image as a NumPy array.
		crop_box (x1, y1, x2, y2): Tuple containing crop box coordinates (left, top, right, bottom).

	Returns:
		np.array: Cropped Image
	"""

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

def tile_image(image, num_rows, num_cols):
	"""
	Tile an image with the specified grid size.

	Args:
		image (np.array): Input image as a NumPy array.
		num_rows (int): number of rows.
		num_cols (int): number of columns.

	Returns:
		List[np.array]: list of tiled images.
	"""
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

def draw_bounding_box(image, bounding_box):
	"""
	Draw a green bounding box on an image.

	Args:
		image (np.array): Input image as a NumPy array.
		bounding_box (x1, y1, x2, y2): bounding box coordinates in the (left, top, right, bottom) format.

	Returns:
		None
	"""
	x1, y1, x2, y2 = bounding_box
	point1 = (round(x1), round(y1))
	point2 = (round(x2), round(y2))

	green = (0, 255, 0)
	
	cv2.rectangle(image, point1, point2, color=green, thickness=2)

def show_image_non_block(image, name='Image'):
	'''
	Show image without blocking the current thread.
	The image is opened by another thread running `show_image` which terminates when the window is closed.
	
	Args:
		image (np.array): Input image as a NumPy array.
		name (str): Window name.
  
	Returns:
		None
	'''
	
	image_thread = threading.Thread(target=show_image, args=(image,name))
	image_thread.start()

__IMAGE_LOCK = Lock()
def show_image(image, name='Image'):
	'''
	Show image and blocks the thread until the window is closed.
	
	Args:
		image (np.array): Input image as a NumPy array.
		name (str): Window name.
  
	Returns:
		None
	'''

	root = tk.Tk()
	screen_height = root.winfo_screenheight()
	root.destroy()
	new_height = screen_height * 3 / 4
 
	# get the width/height ratio
	aspect_ratio = float(image.shape[1]) / float(image.shape[0])
	new_height = int(new_height)
	new_width = int(aspect_ratio * new_height)

	global __IMAGE_LOCK
	with __IMAGE_LOCK:
		cv2.namedWindow(name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
		cv2.resizeWindow(name, (new_width, new_height))
		cv2.imshow(name, image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()