##takes in user input for x1,y1,x2,y2
def main():
	x1 = input("Enter x1: ")
	y1 = input("Enter y1: ")
	x2 = input("Enter x2: ")
	y2 = input("Enter y2: ")
	print(return_num(int(x1),int(y1),int(x2),int(y2)))

# checks if the user input is between a certain range and returns appropriate position

def check_x_pos(x1,x2):
	if x1 >= 0 and x2 <= 46:
		return 1
	else:
		return 0

def check_y_pos(y1,y2):
	if y1 >= 184 and y2 <= 223:
		return 5
	else:
		return 0

# the defined calculation for the box number
def return_num(x1, y1, x2, y2):
	return 8 * check_y_pos(y1, y2) + check_x_pos(x1, x2)

main()
