# checks if the user input is between a certain range and returns appropriate position
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
# the defined calculation for the box number
def get_chip_slot_number(bounding_box):
	x1, y1, x2, y2 = bounding_box
	row = get_chip_row(y1, y2)
	if row == -1:
		return None
	col = get_chip_col(x1, x2)
	if col == -1:
		return None
	return 8 * row + col + 1
