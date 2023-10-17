__ROW_COUNT = 8

def __get_chip_col(x1, x2):
	'''
	This method translated x coordinates into the appropriate chip column number.
	We start from 0 and go to 7. This is to be the correct format for the equation later

	Args:
		x1 (int) : the x coordinate of the top left corner of the bounding box
		x2 (int) : the x coordinate of the bottom right corner of the bounding box

	Returns:
		int : the column number of the chip, or -1 for invalid column
	'''
	if x1 < x2 and x1 >=0 and x2 >= 0:
		x_center = (x1 + x2) / 2
		if x_center >= 4 and x_center <= 37:
			return 0
		if x_center >= 56 and x_center <= 86:
			return 1
		if x_center >= 105 and x_center <= 135:
			return 2
		if x_center >= 155 and x_center <= 182:
			return 3
		if x_center >= 204 and x_center <= 233:
			return 4
		if x_center >= 253 and x_center <= 284:
			return 5
		if x_center >= 302 and x_center <= 329:
			return 6
		if x_center >= 351 and x_center <= 381:
			return 7
		else:
			print(f"X-center out of bounds: {x_center}")
			return -1
	else:
		print("Invalid X-coordinates")
		return -1

def __get_chip_row(y1, y2):
	'''
	This method translated y coordinates into the appropriate chip row number.
	We start from 0 and go to 5. This is to be the correct format for the equation later

	Args:
		y1 (int) : the y coordinate of the top left corner of the bounding box
		y2 (int) : the y coordinate of the bottom right corner of the bounding box

	Returns:
		int : the row number of the chip, or -1 for invalid row
	'''
	if y1 < y2 and y1 >=0 and y2 >= 0:
		y_center = (y1 + y2) / 2
		if y_center >= 0 and y_center <= 40:
			return 0
		if y_center >= 39 and y_center <= 76:
			return 1
		if y_center >= 70 and y_center <= 113:
			return 2
		if y_center >= 112 and y_center <= 151:
			return 3
		if y_center >= 150 and y_center <= 189:
			return 4
		if y_center >= 184 and y_center <= 223:
			return 5
		else:
			print(f"Y-center out of bounds: {y_center}")
			return -1
	else:
		print("Invalid Y-coordinates")
		return -1

def get_chip_slot_number(bounding_box):
	'''
	This method takes in the bounding box of a chip and returns the chip number defined by the cobot code.

	Args:
		bounding_box (list) : the bounding box of the chip

	Returns:
		int : the chip number, or None for invalid chip
	'''
	x1, y1, x2, y2 = bounding_box
	row = __get_chip_row(y1, y2)
	if row == -1:
		return None

	col = __get_chip_col(x1, x2)
	if col == -1:
		return None

	return __ROW_COUNT * col + row + 1
