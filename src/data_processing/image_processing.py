import cv2

def crop_image(image, crop_box):
	"""
	Crops an image based on the specified crop box.

	Args:
		image (np.array): Input image as a NumPy array.
		crop_box (x1, y1, x2, y2): Tuple containing crop box coordinates (left, top, right, bottom).

	Returns:
		np.array: Cropped Image
	"""
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
	x1_int = round(x1)
	y1_int = round(y1)
	x2_int = round(x2)
	y2_int = round(y2)

	green = (0, 255, 0)
	cv2.rectangle(image, (x1_int, y1_int), (x2_int, y2_int), green, 2)
