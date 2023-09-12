import cv2

def crop_image(image, crop_box):
	"""
	Crops an image based on the specified crop box.

	Args:
		image (np.array): Input image as a NumPy array.
		crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).

	Returns:
		Cropped Image
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
		List of tiled images
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
		bounding_box (tuple): bounding box coordinates in the (left, top, right, bottom) format.

	Returns:
		List of tiled images
	"""
	x1, y1, x2, y2 = bounding_box
	green = (0, 255, 0)
	cv2.rectangle(image, (x1, y1), (x2, y2), green, 2)
