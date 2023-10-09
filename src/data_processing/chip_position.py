def get_chip_col(x1,x2):
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
		xcen = (x1 + x2) / 2 #finds the center of the chips x coords
		if xcen >= 4 and xcen <= 37:
			return 0
		if xcen >= 56 and xcen <= 86:
			return 1
		if xcen >= 105 and xcen <= 135:
			return 2
		if xcen >= 155 and xcen <= 182:
			return 3
		if xcen >= 204 and xcen <= 233:
			return 4
		if xcen >= 253 and xcen <= 284:
			return 5
		if xcen >= 302 and xcen <= 329:
			return 6
		if xcen >= 351 and xcen <= 381:
			return 7
		else:
			print(f"X-center out of bounds: {xcen}")
			return -1
	else:
		print("Invalid X-coordinates")
		return -1

def get_chip_row(y1,y2):
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
		ycen = (y1 + y2) / 2 #finds the center of the chips y coords
		if ycen >= 0 and ycen <= 40:
			return 0
		if ycen >= 39 and ycen <= 76:
			return 1
		if ycen >= 70 and ycen <= 113:
			return 2
		if ycen >= 112 and ycen <= 151:
			return 3
		if ycen >= 150 and ycen <= 189:
			return 4
		if ycen >= 184 and ycen <= 223:
			return 5
		else:
			print(f"Y-center out of bounds: {ycen}")
			return -1
	else:
		print("Invalid Y-coordinates")
		return -1

def get_chip_slot_number(bounding_box):
	'''
	This method takes in the bounding box of a chip and returns the chip number
	The chip number is calculated by the equation (8 * row) + col + 1
	The above formulae was devised as the best way to communicate positions to the ROS team

	Args:
		bounding_box (list) : the bounding box of the chip

	Returns:
		int : the chip number, or None for invalid chip
	'''
	x1, y1, x2, y2 = bounding_box
	row = get_chip_row(y1, y2)
	if row == -1:
		return None
	col = get_chip_col(x1, x2)
	if col == -1:
		return None
	return 6 * col + row + 1
