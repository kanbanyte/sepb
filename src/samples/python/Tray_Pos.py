tray1 = "empty"
tray2 = "empty"
assembly = "empty"

# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(x1, y1, x2, y2):
	if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
		assembly = "has tray"
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		tray2 = "has tray"
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		tray1 = "has tray"

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

print(tray_class_move(5, 147, 330, 397, "empty"))
