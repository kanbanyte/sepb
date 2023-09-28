'''
This method translated x coordinates into the appropriate chip column number.
We start from 0 and go to 7. This is to be the correct format for the equation later

	args:
		x1 (int) : the x coordinate of the top left corner of the bounding box
		x2 (int) : the x coordinate of the bottom right corner of the bounding box

	returns:
		int : the column number of the chip, or -1 for invalid column
'''

def get_chip_col(x1,x2):
	if x1 >= 4 and x2 <= 32:
		return 0
	if x1 >= 61 and x2 <= 81:
		return 1
	if x1 >= 110 and x2 <= 130:
		return 2
	if x1 >= 160 and x2 <= 177:
		return 3
	if x1 >= 209 and x2 <= 228:
		return 4
	if x1 >= 258 and x2 <= 274:
		return 5
	if x1 >= 307 and x2 <= 324:
		return 6
	if x1 >= 356 and x2 <= 376:
		return 7
	else:
		return -1

'''
This method translated y coordinates into the appropriate chip row number.
We start from 0 and go to 5. This is to be the correct format for the equation later

	args:
		y1 (int) : the y coordinate of the top left corner of the bounding box
		y2 (int) : the y coordinate of the bottom right corner of the bounding box

	returns:
		int : the row number of the chip, or -1 for invalid row
'''
def get_chip_row(y1,y2):
	if y1 >= 0 and y2 <= 40:
		return 0
	if y1 >= 39 and y2 <= 76:
		return 1
	if y1 >= 70 and y2 <= 113:
		return 2
	if y1 >= 112 and y2 <= 151:
		return 3
	if y1 >= 150 and y2 <= 189:
		return 4
	if y1 >= 184 and y2 <= 223:
		return 5
	else:
		return -1

'''
This method takes in the bounding box of a chip and returns the chip number
The chip number is calculated by the equation (8 * row) + col + 1
The above formulae was devised as the best way to communicate positions to the ROS team

	args:
		bounding_box (list) : the bounding box of the chip

	returns:
		int : the chip number, or None for invalid chip
'''
def get_chip_slot_number(bounding_box):
	x1, y1, x2, y2 = bounding_box
	row = get_chip_row(y1, y2)
	if row == -1:
		return None
	col = get_chip_col(x1, x2)
	if col == -1:
		return None
	return 8 * col + row + 1
