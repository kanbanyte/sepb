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
class TrayState(Enum):
	not_present = 0
	full = 1
	empty = 2
	partially_full = 3

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
	tray2 = "tray 2"
	tray1 = "tray 1"
	assembly = "assembly"

	# dictionary of tray states
	tray_states = {
		tray1: TrayState.not_present,
		tray2: TrayState.not_present,
		assembly: TrayState.not_present
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
					tray_states.update({assembly: TrayState.empty})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({assembly: TrayState.full})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({assembly: TrayState.partially_full})
				else:
					tray_states.update({assembly: TrayState.not_present})
			elif x1 >= 340 and y1 >= 0 and x2 <= 720 and y2 <= 265:
				if model.classes[class_index].lower() == "empty":
					tray_states.update({tray2: TrayState.empty})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({tray2: TrayState.full})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({tray2: TrayState.partially_full})
				else:
					tray_states.update({tray2: TrayState.not_present})
			elif x1 >= 340 and y1 >= 288 and x2 <= 720 and y2 <= 590:
				if model.classes[class_index].lower() == "empty":
					tray_states.update({tray1: TrayState.empty})
				elif model.classes[class_index].lower() == "full":
					tray_states.update({tray1: TrayState.full})
				elif model.classes[class_index].lower() == "partially full":
					tray_states.update({tray1: TrayState.partially_full})
				else:
					tray_states.update({tray1: TrayState.not_present})
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
	# TODO: clean this
	print(f" states:{tray_states}")
	if tray_states.get("assembly") == TrayState.not_present:
		if tray_states.get("tray 1") == TrayState.full:
			return TrayMovement.move_tray1_assembly
		elif tray_states.get("tray 2") == TrayState.full:
			return TrayMovement.move_tray2_assembly
		else:
			return TrayMovement.both_empty
	elif tray_states.get("assembly") == TrayState.empty:
		if tray_states.get("tray 1") == TrayState.not_present:
			return TrayMovement.move_assembly_tray1
		elif tray_states.get("tray 2") == TrayState.not_present:
			return TrayMovement.move_assembly_tray2
		else:
			if tray_states.get("tray 1") == TrayState.full:
				return TrayMovement.move_tray1_assembly
			elif tray_states.get("tray 2") == TrayState.full:
				return TrayMovement.move_tray2_assembly
			else:
				return TrayMovement.no_move
	# elif tray_states.get("tray 1") == TrayState.empty and tray_states.get("tray 2") == TrayState.empty:
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
