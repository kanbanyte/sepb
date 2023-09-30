from enum import Enum

'''
Constants representing the names of states of the tray movement and presence
'''
__ASSEMBLY_PRESENT = "assembly_present"
__ASSEMBLY_MOVABLE =  "assembly_movable"
__TRAY1_PRESENT =  "tray1_present"
__TRAY1_MOVABLE =  "tray1_movable"
__TRAY2_PRESENT =  "tray2_present"
__TRAY2_MOVABLE = "tray2_movable"

'''
State dictionary showing the presence and movability of the tray.
Note that assembly, tray1 and tray2 indicates the 3 slots a tray can be found in, not the trays themselves.
'''
__tray_states = {
	__ASSEMBLY_PRESENT: False,
	__ASSEMBLY_MOVABLE: False,
	__TRAY1_PRESENT: False,
	__TRAY1_MOVABLE: False,
	__TRAY2_PRESENT: False,
	__TRAY2_MOVABLE: False
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

def update_tray_positions(bounding_box, tray_class):
	'''
		This method takes in a bounding box and tray class and returns the position of the tray.
		It also updates the internal state dictionary with the tray position and whether it is in a moveable state, which is then used to determine the next movement.

		Args:
  			bounding_box (list[int]): The bounding box of the tray.
			tray_class (str): The class of the tray.

		Returns:
			TrayPos(): an enum value representing the tray position.
	'''

	x1, y1, x2, y2 = bounding_box

	# check the tray presence in the assembly slot
	if x1 >= 0 and y1 >= 140 and x2 <= 335 and y2 <= 400:
		__tray_states.update({__ASSEMBLY_PRESENT: True})
		if tray_class.lower() == "empty":
			__tray_states.update({__ASSEMBLY_MOVABLE: True})
		return TrayPos.assembly

	# check the tray presence in the second slot (upper right from operator)
	if x1 >= 340 and y1 >= 0 and x2 <= 672 and y2 <= 265:
		__tray_states.update({__TRAY2_PRESENT: True})
		if tray_class.lower() == "full":
			__tray_states.update({__TRAY2_MOVABLE: True})
		return TrayPos.tray2

	# check the tray presence in the first slot (lower right from operator)
	if x1 >= 340 and y1 >= 288 and x2 <= 685 and y2 <= 575:
		__tray_states.update({__TRAY1_PRESENT: True})
		if tray_class.lower() == "full":
			__tray_states.update({__TRAY1_MOVABLE: True})
		return TrayPos.tray1


def determine_tray_movement():
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

	if not __tray_states.get(__ASSEMBLY_PRESENT):
		if __tray_states.get(__TRAY1_MOVABLE):
			__tray_states.update({__TRAY1_PRESENT: False})
			__tray_states.update({__TRAY1_MOVABLE: False})
			return TrayMovement.move_tray1_assembly

		if __tray_states.get(__TRAY2_MOVABLE):
			__tray_states.update({__TRAY2_MOVABLE: False})
			__tray_states.update({__TRAY2_PRESENT: False})
			return TrayMovement.move_tray2_assembly

	if __tray_states.get(__ASSEMBLY_PRESENT) and __tray_states.get(__ASSEMBLY_MOVABLE):
		if not __tray_states.get(__TRAY1_PRESENT):
			__tray_states.update({__ASSEMBLY_PRESENT: False})
			__tray_states.update({__ASSEMBLY_MOVABLE: False})
			return TrayMovement.move_assembly_tray1

		if not __tray_states.get(__TRAY2_PRESENT):
			__tray_states.update({__ASSEMBLY_PRESENT: False})
			__tray_states.update({__ASSEMBLY_MOVABLE: False})
			return TrayMovement.move_assembly_tray2

	return TrayMovement.no_move

# #endregion

# this is test code

# calls the functions from the class, creating example bounding boxes and tray classes
# get_position_from_bounding_box([0, 144, 335, 400], "empty")
# get_position_from_bounding_box([340, 288, 685, 575], "full")
# # calls the check_move function, this returns the relevant move text
# print(check_move())
def get_states(detections, model):
	class Traystate(enum):
		not_present = 0
		full = 1
		empty = 2
		part_full = 3

	__tray2 = "tray 2"
	__tray1 = "tray 1"
	__assembly = "assembly"
	__tray_states = {
		__tray1: Traystate.not_present,
		__tray2: Traystate.not_present,
		__assembly: Traystate.not_present
	}

	for class_index, detected_objects in detections.items():
		for i, detected_tray in enumerate(detected_objects):
			x1, y1, x2, y2 = detected_tray.bounding_box
			if x1 >= 0 and y1 >= 140 and x2 <= 335 and y2 <= 400:
				if model.classes[class_index].lower() == "empty":
					__tray_states.update({__assembly: Traystate.empty})
				elif model.classes[class_index].lower() == "full":
					__tray_states.update({__assembly: Traystate.full})
				elif model.classes[class_index].lower() == "partially full":
					__tray_states.update({__assembly: Traystate.part_full})
				else:
					__tray_states.update({__assembly: Traystate.not_present})
			elif x1 >= 340 and y1 >= 0 and x2 <= 672 and y2 <= 265:
				if model.classes[class_index].lower() == "empty":
					__tray_states.update({__tray2: Traystate.empty})
				elif model.classes[class_index].lower() == "full":
					__tray_states.update({__tray2: Traystate.full})
				elif model.classes[class_index].lower() == "partially full":
					__tray_states.update({__tray2: Traystate.part_full})
				else:
					__tray_states.update({__tray2: Traystate.not_present})
			elif x1 >= 340 and y1 >= 288 and x2 <= 685 and y2 <= 575:
				if model.classes[class_index].lower() == "empty":
					__tray_states.update({__tray1: Traystate.empty})
				elif model.classes[class_index].lower() == "full":
					__tray_states.update({__tray1: Traystate.full})
				elif model.classes[class_index].lower() == "partially full":
					__tray_states.update({__tray1: Traystate.part_full})
				else:
					__tray_states.update({__tray1: Traystate.not_present})
	return __tray_states


def determine_move(detections, model):
	pass
