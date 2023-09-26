tray1 = False
tray1_move = False
tray2 = False
tray2_move = False
assembly = False
assembly_move = False

# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(x1, y1, x2, y2, tray_class):
	if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
		assembly = True
		if tray_class == "empty":
			assembly_move = True
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		tray2 = True
		if tray_class == "full":
			tray2_move = True
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		tray1 = True
		if tray_class == "full":
			tray1_move = True

#determine class and if the tray needs to be moved
def tray_class_move(x1, y1, x2, y2, Tray_class):
	_temp = get_position_from_bounding_box(x1, y1, x2, y2)
	print(_temp)
	def switch_assembly(t):
		if t == "Tray Pos is Assembly" and Tray_class == "full":
			return "do not move"
		elif t == "Tray Pos is Assembly" and Tray_class == "partially_full":
			return "do not move"
		elif t == "Tray Pos is Assembly" and Tray_class == "empty":
			return "move empty slot asap"
		else:
			return "clear"

	def switch_t1(t):
		if t == "Tray Pos is tray 2" and Tray_class == "full":
			return "move to assembly"
		elif t == "Tray Pos is tray 2" and Tray_class == "partially_full":
			return "do not move"
		elif t == "Tray Pos is tray 2" and Tray_class == "empty":
			return "do not move"
		else:
			return "clear"

	def switch_t2(t):
		if t == "Tray Pos is tray 1" and Tray_class == "full":
			return "move to assembly"
		elif t == "Tray Pos is tray 1" and Tray_class == "partially_full":
			return "do not move"
		elif t == "Tray Pos is tray 1" and Tray_class == "empty":
			return "do not move"
		else:
			return "clear"

	if switch_assembly(_temp) == "move empty slot asap" and switch_t1(_temp) == "clear":
		return "move to tray 1"
	if switch_assembly(_temp) == "move empty slot asap" and switch_t2(_temp) == "clear":
		return "move to tray 2"

	if switch_t1(_temp) == "move to assembly" and switch_assembly(_temp) == "clear":
		return "move to assembly"
	if switch_t2(_temp) == "move to assembly" and switch_assembly(_temp) == "clear":
		return "move to assembly"


def check_move(tray_class):
	if assembly == "empty":
		if tray1 == "full":
			tray1 = "empty"
			return "move to assembly"
		elif tray2 == "full":
			tray2 = "empty"
			return "move to assembly"
	if assembly == "has tray" and tray_class == "empty":
		if tray1 == "empty":
			assembly = "empty"
			return "move to tray 1"
		elif tray2 == "empty":
			assembly = "empty"
			return "move to tray 2"


print(tray_class_move(5, 147, 330, 397, "empty"))
