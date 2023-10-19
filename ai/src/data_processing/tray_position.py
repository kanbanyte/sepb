from enum import Enum

'''
Enum of each Tray positions possible states
'''
class TrayState(Enum):
	ABSENT = 0
	FULL = 1
	EMPTY = 2
	PARTIALLY_FULL = 3

'''
Enum representing the best cobot move after detecting the trays.
'''
class CobotMovement(Enum):
	# Do nothing
	NONE = 0

	# Move assembly to tray 1
	ASSEMBLY_TO_TRAY1 = 1

	# Move assembly to tray 2
	ASSEMBLY_TO_TRAY2 = 2

	# Move tray 1 to assembly
	TRAY1_TO_ASSEMBLY = 3

	# Move tray 2 to assembly
	TRAY2_TO_ASSEMBLY = 4

	# Starting loading tray 1 when it is empty
	START_TRAY1_LOAD = 5

	# Starting loading tray 2 when it is empty
	START_TRAY2_LOAD = 6

	# Continue loading tray 1 when it is partially full
	CONTINUE_TRAY1_LOAD = 7

	# Continue loading tray 2 when it is partially full
	CONTINUE_TRAY2_LOAD = 8

# Tray names in string for reference later
__TRAY_1 = "tray 1"
__TRAY_2 = "tray 2"
__ASSEMBLY = "assembly"

def __convert_class_to_state(class_name):
	lowered_name = class_name.lower()
	if lowered_name == "empty":
		return TrayState.EMPTY

	if lowered_name == "partially full":
		return TrayState.PARTIALLY_FULL

	if lowered_name == "full":
		return TrayState.FULL

	raise ValueError(f"Invalid class name from Tray Detection Model: {class_name}")

def __convert_bounding_boxes_to_states(detections, model):
	'''
	Returns a dictionary of the tray states

	Args:
		detections (dict): dictionary of detected objects
		model (ObjectDetectionModel): model used to get the detections

	Returns:
		dict[int, list[DetectedObjects]]: dictionary of tray states
	'''

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
		tray_state = __convert_class_to_state(model.classes[class_index])

		for detected_tray in detected_objects:
			print(detected_tray.bounding_box)
			x1, y1, x2, y2 = detected_tray.bounding_box
			x_center = round((x1 + x2) / 2)
			y_center = round((y1 + y2) / 2)

			# the crop box is around 700 pixels in width,
			# and there is only one tray on the left from the POV of the human operator
			if x_center < 350:
				tray_states.update({__ASSEMBLY: tray_state})
				continue

			# the crop box is around 600 pixels in height,
			# and there are 2 trays on the right from the POV of the human operator
			if y_center > 300:
				tray_states.update({__TRAY_1: tray_state})
			else:
				tray_states.update({__TRAY_2: tray_state})

	return tray_states

def __get_best_move(tray_states):
	'''
	Returns the movement of the tray by taking in a dictionary of states

	Args:
		tray_states (dict): dictionary of tray states

	Returns:
		TrayMovement: movement of the tray
	'''

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
		return CobotMovement.CONTINUE_TRAY2_LOAD

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
	tray_states = __convert_bounding_boxes_to_states(detections, model)
	print(f"Tray States: {tray_states}")
	return __get_best_move(tray_states)
