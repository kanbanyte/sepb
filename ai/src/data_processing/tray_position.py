from enum import Enum

'''
Enum of possible tray movements.
'''
class TrayMovement(Enum):
	ASSEMBLY_TO_TRAY1 = 1
	ASSEMBLY_TO_TRAY2 = 2
	TRAY1_TO_ASSEMBLY = 3
	TRAY2_TO_ASSEMBLY = 4
	NONE = 5
	# TODO: RENAME THIS VALUE AND FIX FAILING UNIT TEST
	both_empty = 6

'''
Enum of each Tray positions possible states
'''
class TrayState(Enum):
	ABSENT = 0
	FULL = 1
	EMPTY = 2
	PARTIALLY_FULL = 3

# Tray names in string for reference later
__TRAY_1 = "tray 1"
__TRAY_2 = "tray 2"
__ASSEMBLY = "assembly"

def __get_states(detections, model):
	'''
	Returns a dictionary of the tray states

	Args:
		detections (dict): dictionary of detected objects

	Returns:
		dict: dictionary of tray states
	'''

	# dictionary of tray states
	tray_states = {
		__TRAY_1: TrayState.ABSENT,
		__TRAY_2: TrayState.ABSENT,
		__ASSEMBLY: TrayState.ABSENT
	}

	# detections is a dictionary of detected objects, in this format:
	# dict_items([
	# 	(0, [DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))]),
	# 	(1, [DetectedObject(confidence=0.950691819190979, bounding_box=(344, 293, 675, 574))])
	# ])
	for class_index, detected_objects in detections.items():
		# detected_objects is a list of DetectedObject, in this format:
		# [DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))]
		for detected_tray in detected_objects:
			# detected_tray is a DetectedObject, in this format:
			# DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))
			x1, y1, x2, y2 = detected_tray.bounding_box
			if x1 >= 0 and y1 >= 140 and x2 <= 400 and y2 <= 420:
				if model.classes[class_index].lower() == "empty":
					tray_states.update({__ASSEMBLY: TrayState.EMPTY})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({__ASSEMBLY: TrayState.FULL})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({__ASSEMBLY: TrayState.PARTIALLY_FULL})
				else:
					tray_states.update({__ASSEMBLY: TrayState.ABSENT})
			elif x1 >= 340 and y1 >= 0 and x2 <= 720 and y2 <= 265:
				if model.classes[class_index].lower() == "empty":
					tray_states.update({__TRAY_2: TrayState.EMPTY})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({__TRAY_2: TrayState.FULL})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({__TRAY_2: TrayState.PARTIALLY_FULL})
				else:
					tray_states.update({__TRAY_2: TrayState.ABSENT})
			elif x1 >= 340 and y1 >= 288 and x2 <= 720 and y2 <= 590:
				if model.classes[class_index].lower() == "empty":
					tray_states.update({__TRAY_1: TrayState.EMPTY})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({__TRAY_1: TrayState.FULL})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({__TRAY_1: TrayState.PARTIALLY_FULL})
				else:
					tray_states.update({__TRAY_1: TrayState.ABSENT})
			else:
				print("Tray Position Conversion Error: Out of bounds position detected")

	return tray_states

def __get_movement(tray_states):
	'''
	Returns the movement of the tray by taking in a dictionary of states

	Args:
		tray_states (dict): dictionary of tray states

	Returns:
		TrayMovement: movement of the tray
	'''
	print(f"Tray states: {tray_states}")
	if tray_states.get(__ASSEMBLY) == TrayState.ABSENT:
		if tray_states.get(__TRAY_1) == TrayState.FULL:
			return TrayMovement.TRAY1_TO_ASSEMBLY

		if tray_states.get(__TRAY_2) == TrayState.FULL:
			return TrayMovement.TRAY2_TO_ASSEMBLY

		return TrayMovement.both_empty

	if tray_states.get(__ASSEMBLY) == TrayState.EMPTY:
		if tray_states.get(__TRAY_1) == TrayState.ABSENT:
			return TrayMovement.ASSEMBLY_TO_TRAY1

		if tray_states.get(__TRAY_2) == TrayState.ABSENT:
			return TrayMovement.ASSEMBLY_TO_TRAY2

		if tray_states.get(__TRAY_1) == TrayState.FULL:
			return TrayMovement.TRAY1_TO_ASSEMBLY

		if tray_states.get(__TRAY_2) == TrayState.FULL:
			return TrayMovement.TRAY2_TO_ASSEMBLY

		return TrayMovement.NONE
	# elif tray_states.get("tray 1") == TrayState.empty and tray_states.get("tray 2") == TrayState.empty:
	# 	return TrayMovement.both_empty

	return TrayMovement.NONE

class CobotMovement(Enum):
	NONE = 0
	ASSEMBLY_TO_TRAY1 = 1
	ASSEMBLY_TO_TRAY2 = 2
	TRAY1_TO_ASSEMBLY = 3
	TRAY2_TO_ASSEMBLY = 4
	START_TRAY1_LOAD = 5
	START_TRAY2_LOAD = 6
	CONTINUE_TRAY1_LOAD = 7
	CONTINUE_TRAY2_LOAD = 8

def __get_cobot_move(tray_states):
	'''
	Returns the movement of the tray by taking in a dictionary of states

	Args:
		tray_states (dict): dictionary of tray states

	Returns:
		TrayMovement: movement of the tray
	'''
	print(f"Tray states: {tray_states}")

	# if assembly tray is empty, we move it back for future loading
	if tray_states.get(__ASSEMBLY) == TrayState.EMPTY:
		if tray_states.get(__TRAY_1) == TrayState.ABSENT:
			return CobotMovement.ASSEMBLY_TO_TRAY1

		if tray_states.get(__TRAY_2) == TrayState.ABSENT:
			return CobotMovement.ASSEMBLY_TO_TRAY2

		return CobotMovement.NONE

	# if assembly tray is absent, we move any full tray there
	if tray_states.get(__ASSEMBLY) == TrayState.ABSENT:
		if tray_states.get(__TRAY_1) == TrayState.FULL:
			return CobotMovement.TRAY1_TO_ASSEMBLY

		if tray_states.get(__TRAY_2) == TrayState.FULL:
			return CobotMovement.TRAY2_TO_ASSEMBLY

	# if tray 1 or 2 are empty, we start loading that tray
	if tray_states.get(__TRAY_1) == TrayState.EMPTY:
		return CobotMovement.START_TRAY1_LOAD

	if tray_states.get(__TRAY_2) == TrayState.EMPTY:
		return CobotMovement.START_TRAY2_LOAD

	# if tray 1 or 2 are empty, we continue loading that tray
	if tray_states.get(__TRAY_1) == TrayState.PARTIALLY_FULL:
		return CobotMovement.CONTINUE_TRAY1_LOAD

	if tray_states.get(__TRAY_2) == TrayState.PARTIALLY_FULL:
		return CobotMovement.CONTINUE_TRAY1_LOAD

	return CobotMovement.NONE

def determine_move(detections, model):
	'''
		Returns the movement of the tray by taking in a dictionary of detected objects and a model

		Args:
			detections (dict): dictionary of detected objects
			model (ObjectDetectionModel): model used to detect objects
		Returns:
			TrayMovement: movement of the tray
	'''
	tray_states = __get_states(detections, model)
	tray_states = __get_cobot_move(detections, model)
	return __get_movement(tray_states)
