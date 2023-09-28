from enum import Enum

'''
Constants representing the names of states of the tray movement and presence
'''
ASSEMBLY_PRESENT = "assembly_present"
ASSEMBLY_MOVABLE =  "assembly_movable"
TRAY1_PRESENT =  "tray1_present"
TRAY1_MOVABLE =  "tray1_movable"
TRAY2_PRESENT =  "tray2_present"
TRAY2_MOVABLE = "tray2_movable"

'''
State dictionary showing the presence and movability of the tray.
Note that assembly, tray1 and tray2 indicates the 3 slots a tray can be found in, not the trays themselves.
'''
dict = {
	ASSEMBLY_PRESENT: False,
	ASSEMBLY_MOVABLE: False,
	TRAY1_PRESENT: False,
	TRAY1_MOVABLE: False,
	TRAY2_PRESENT: False,
	TRAY2_MOVABLE: False
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


# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(bounding_box, tray_class):
	'''
		This method takes in a bounding box and tray class and returns the position of the tray.
		It also updates the dictionary with the tray position and whether it is in a moveable state.
		It does this by checking the bounding box against a range of values.
		The tray class is used to determine whether the tray is full or empty.
		The tray class is not used to determine the position of the tray.
		The tray class is used to determine whether the tray is in a moveable state.

		Args: 
  			bounding_box (list[int]): The bounding box of the tray.
			tray_class (str): The class of the tray.

		Returns:
			TrayPos(): an enum value representing the tray position.
	'''
	x1, y1, x2, y2 = bounding_box
	if x1 >= 0 and y1 >= 140 or x2 <= 335 or y2 <= 400:
		dict.update({ASSEMBLY_PRESENT: True})
		if tray_class.lower() == "empty":
			dict.update({ASSEMBLY_MOVABLE: True})
		return TrayPos.assembly
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		dict.update({TRAY2_PRESENT: True})
		if tray_class.lower() == "full":
			dict.update({TRAY2_MOVABLE: True})
		return TrayPos.tray2
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		dict.update({TRAY1_PRESENT: True})
		if tray_class.lower() == "full":
			dict.update({TRAY1_MOVABLE: True})
		return TrayPos.tray1


def check_move():
	'''
		This method checks the state of the trays and returns a move.
		It does this by checking the dictionary for the state of the trays.
		If the tray is in a moveable state it will return a move.
		If the tray is not in a moveable state it will return no_move.

		Args: 
  			None

		Returns:
			TrayMovement: an enum value representing the desired tray movement.

	'''
	if not dict.get(ASSEMBLY_PRESENT):
		if dict.get(TRAY1_MOVABLE):
			dict.update({TRAY1_PRESENT: False})
			dict.update({TRAY1_MOVABLE: False})
			return TrayMovement.move_tray1_assembly
		
		if dict.get(TRAY2_MOVABLE):
			dict.update({TRAY2_MOVABLE: False})
			dict.update({TRAY2_PRESENT: False})
			return TrayMovement.move_tray2_assembly

	if dict.get(ASSEMBLY_PRESENT) and dict.get(ASSEMBLY_MOVABLE):
		if not dict.get(TRAY1_PRESENT):
			dict.update({ASSEMBLY_PRESENT: False})
			dict.update({ASSEMBLY_MOVABLE: False})
			return TrayMovement.move_assembly_tray1
		
		if not dict.get(TRAY2_PRESENT):
			dict.update({ASSEMBLY_PRESENT: False})
			dict.update({ASSEMBLY_MOVABLE: False})
			return TrayMovement.move_assembly_tray2

	return TrayMovement.no_move

# #endregion

# this is test code

# calls the functions from the class, creating example bounding boxes and tray classes
# get_position_from_bounding_box([0, 144, 335, 400], "empty")
# get_position_from_bounding_box([340, 288, 685, 575], "full")
# # calls the check_move function, this returns the relevant move text
# print(check_move())
