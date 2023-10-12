from enum import Enum

'''
Enum of possible tray movements
move indicates a movement from position one, labelled first to position two labelled second
move_assembly_tray1 indicates a movement from assembly to tray 1, and so forth
no_move indicates no movement
'''
class TrayMovement(Enum):
	move_assembly_tray1 = 1
	move_assembly_tray2 = 2
	move_tray1_assembly = 3
	move_tray2_assembly = 4
	no_move = 5
	both_empty = 6
'''
Enum of each Tray positions possible states
'''
class Traystate(Enum):
	not_present = 0
	full = 1
	empty = 2
	part_full = 3


def __get_states(detections, model):
	'''
		Returns a dictionary of the tray states
		Args:
			detections (dict): dictionary of detected objects
			model (ObjectDetectionModel): model used to detect objects
		Returns:
			dict: dictionary of tray states
	'''
	# Tray names in string for reference later
	__tray2 = "tray 2"
	__tray1 = "tray 1"
	__assembly = "assembly"
	# dictionary of tray states
	__tray_states = {
		__tray1: Traystate.not_present,
		__tray2: Traystate.not_present,
		__assembly: Traystate.not_present
	}

	# detections is a dictionary of detected objects, in this format:
	# dict_items([(0, [DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))]), (1, [DetectedObject(confidence=0.950691819190979, bounding_box=(344, 293, 675, 574))])])
	for class_index, detected_objects in detections.items():
		# detected_objects is a list of DetectedObject, in this format:
		# [DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))]
		for detected_tray in detected_objects:
			# detected_tray is a DetectedObject, in this format:
			# DetectedObject(confidence=0.9711142182350159, bounding_box=(2, 144, 331, 400))
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

def __get_movement(tray_states):
	'''
	Returns the movement of the tray by taking in a dictionary of states

	Args:
		tray_states (dict): dictionary of tray states
	Returns:
		TrayMovement: movement of the tray
	'''
	# TODO: clean this
	if tray_states.get("assembly") == Traystate.not_present:
		if tray_states.get("tray 1") == Traystate.full:
			return TrayMovement.move_tray1_assembly
		elif tray_states.get("tray 2") == Traystate.full:
			return TrayMovement.move_tray2_assembly
		else:
			return TrayMovement.both_empty
	elif tray_states.get("assembly") == Traystate.empty:
		if tray_states.get("tray 1") == Traystate.not_present:
			return TrayMovement.move_assembly_tray1
		elif tray_states.get("tray 2") == Traystate.not_present:
			return TrayMovement.move_assembly_tray2
		else:
			if tray_states.get("tray 1") == Traystate.full:
				return TrayMovement.move_tray1_assembly
			elif tray_states.get("tray 2") == Traystate.full:
				return TrayMovement.move_tray2_assembly
			else:
				return TrayMovement.no_move
	# elif tray_states.get("tray 1") == Traystate.empty and tray_states.get("tray 2") == Traystate.empty:
	# 	return TrayMovement.both_empty
	else:
		return TrayMovement.no_move

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
	return __get_movement(tray_states)
