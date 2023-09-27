from enum import Enum
'''
This is a dictonary of the trays and their states.
The states are either True or False.
True means that the tray is in a moveable state as denoted by _move
True also means the tray is present.
False is the inverse, the Tray is not in a moveable state or is not present.
'''
dict = {
	"assembly": False,
	"assembly_move": False,
	"tray1": False,
	"tray1_move": False,
	"tray2": False,
	"tray2_move": False
}

'''
enumerations for the tray positions and movements
They will in the format return data in repr() format, this can be changed with .name or .value appended to the end
These enumerations should be subject to changed as needed by the ROS team
'''
class TrayPos(Enum):
	tray1 = 1
	tray2 = 2
	assembly = 3

class TrayMovement(Enum):
	move_assembly_tray1 = 1
	move_assembly_tray2 = 2
	move_tray1_assembly = 3
	move_tray2_assembly = 4
	no_move = 5


'''
This method takes in a bounding box and tray class and returns the position of the tray.
It also updates the dictionary with the tray position and whether it is in a moveable state.
It does this by checking the bounding box against a range of values.
The tray class is used to determine whether the tray is full or empty.
The tray class is not used to determine the position of the tray.
The tray class is used to determine whether the tray is in a moveable state.

    Args: bounding_box (list[int]): The bounding box of the tray.
		  tray_class (str): The class of the tray.

    Returns:
        repr(): A representation of the tray position.
'''
# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(bounding_box, tray_class):
	x1, y1, x2, y2 = bounding_box
	if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
		dict.update({"assembly": True})
		if tray_class.lower() == "empty":
			dict.update({"assembly_move": True})
		return TrayPos.assembly
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		dict.update({"tray2": True})
		if tray_class.lower() == "full":
			dict.update({"tray2_move": True})
		return TrayPos.tray2
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		dict.update({"tray1": True})
		if tray_class.lower() == "full":
			dict.update({"tray1_move": True})
		return TrayPos.tray1

'''
This method checks the state of the trays and returns a move.
It does this by checking the dictionary for the state of the trays.
If the tray is in a moveable state it will return a move.
If the tray is not in a moveable state it will return no_move.

	Args: None

	Returns:
		repr(): A representation of the desired tray movement.

'''
# checks if the trays are in a moveable state and returns a corresponding move
def check_move():
	if dict.get("assembly") == False:
		if dict.get("tray1_move") == True:
			dict.update({"tray1": False})
			dict.update({"tray1_move": False})
			return TrayMovement.move_tray1_assembly
		elif dict.get("tray2_move") == True:
			dict.update({"tray2_move": False})
			dict.update({"tray2": False})
			return TrayMovement.move_tray2_assembly
	if dict.get("assembly") == True and dict.get("assembly_move") == True:
		if dict.get("tray1") == False:
			dict.update({"assembly": False})
			dict.update({"assembly_move": False})
			return TrayMovement.move_assembly_tray1
		elif dict.get("tray2") == False:
			dict.update({"assembly": False})
			dict.update({"assembly_move": False})
			return TrayMovement.move_assembly_tray2
	return TrayMovement.no_move

# #endregion

# this is test code

# calls the functions from the class, creating example bounding boxes and tray classes
# get_position_from_bounding_box([0, 144, 335, 400], "empty")
# get_position_from_bounding_box([340, 288, 685, 575], "full")
# # calls the check_move function, this returns the relevant move text
# print(check_move())