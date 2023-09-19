##takes in user input for x1,y1,x2,y2
def main():
	x1 = input("Enter x1: ")
	y1 = input("Enter y1: ")
	x2 = input("Enter x2: ")
	y2 = input("Enter y2: ")
	print(return_num(int(x1),int(y1),int(x2),int(y2)))

# checks if the user input is between a certain range and returns appropriate position

def check_x_pos(x1,x2):
	if x1 >= 4 and x2 <= 32:
		return 1
	if x1 >= 61 and x2 <= 81:
		return 2
	if x1 >= 110 and x2 <= 130:
		return 3
	if x1 >= 160 and x2 <= 177:
		return 4
	if x1 >= 209 and x2 <= 228:
		return 5
	if x1 >= 258 and x2 <= 274:
		return 6
	if x1 >= 307 and x2 <= 324:
		return 7
	if x1 >= 356 and x2 <= 376:
		return 8
	else:
		print("Error: x1 and x2 are not within range")

def check_y_pos(y1,y2):
	if y1 >= 0 and y2 <= 40:
		return 0
	if y1 >= 39 and y2 <= 76:
		return 1
	if y1 >= 75 and y2 <= 113:
		return 2
	if y1 >= 112 and y2 <= 151:
		return 3
	if y1 >= 150 and y2 <= 189:
		return 4
	if y1 >= 184 and y2 <= 223:
		return 5
	else:
		print("Error: y1 and y2 are not within range")

# the defined calculation for the box number
def return_num(x1, y1, x2, y2):
	return 8 * check_y_pos(y1, y2) + check_x_pos(x1, x2)

main()
