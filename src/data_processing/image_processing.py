def crop_image(image, crop_box):
	"""
	Crops an image based on the specified crop box and saves the cropped image.

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
