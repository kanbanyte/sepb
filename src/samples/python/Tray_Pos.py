class Tray_Pos:
	def __init__(self):
		self.tray1 = False
		self.tray1_move = False
		self.tray2 = False
		self.tray2_move = False
		self.assembly = False
		self.assembly_move = False

	# checks if bounding box is between a certain range and returns a corresponding position
	def get_position_from_bounding_box(self, x1, y1, x2, y2, tray_class):
		if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
			self.assembly = True
			if tray_class == "empty":
				self.assembly_move = True
			return None
		if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
			self.tray2 = True
			if tray_class == "full":
				self.tray2_move = True
			return None
		if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
			self.tray1 = True
			if tray_class == "full":
				self.tray1_move = True
			return None

	#determine class and if the tray needs to be moved
	# def tray_class_move(x1, y1, x2, y2, Tray_class):
	# 	_temp = get_position_from_bounding_box(x1, y1, x2, y2)
	# 	print(_temp)
	# 	def switch_assembly(t):
	# 		if t == "Tray Pos is Assembly" and Tray_class == "full":
	# 			return "do not move"
	# 		elif t == "Tray Pos is Assembly" and Tray_class == "partially_full":
	# 			return "do not move"
	# 		elif t == "Tray Pos is Assembly" and Tray_class == "empty":
	# 			return "move empty slot asap"
	# 		else:
	# 			return "clear"

	# 	def switch_t1(t):
	# 		if t == "Tray Pos is tray 2" and Tray_class == "full":
	# 			return "move to assembly"
	# 		elif t == "Tray Pos is tray 2" and Tray_class == "partially_full":
	# 			return "do not move"
	# 		elif t == "Tray Pos is tray 2" and Tray_class == "empty":
	# 			return "do not move"
	# 		else:
	# 			return "clear"

	# 	def switch_t2(t):
	# 		if t == "Tray Pos is tray 1" and Tray_class == "full":
	# 			return "move to assembly"
	# 		elif t == "Tray Pos is tray 1" and Tray_class == "partially_full":
	# 			return "do not move"
	# 		elif t == "Tray Pos is tray 1" and Tray_class == "empty":
	# 			return "do not move"
	# 		else:
	# 			return "clear"

	# 	if switch_assembly(_temp) == "move empty slot asap" and switch_t1(_temp) == "clear":
	# 		return "move to tray 1"
	# 	if switch_assembly(_temp) == "move empty slot asap" and switch_t2(_temp) == "clear":
	# 		return "move to tray 2"

	# 	if switch_t1(_temp) == "move to assembly" and switch_assembly(_temp) == "clear":
	# 		return "move to assembly"
	# 	if switch_t2(_temp) == "move to assembly" and switch_assembly(_temp) == "clear":
	# 		return "move to assembly"


	def check_move(self):
		if self.assembly == False:
			if self.tray1_move == True:
				self.tray1 = False
				self.tray1_move = False
				return "move tray 1 to assembly"
			elif self.tray2_move == True:
				self.tray2_move = False
				self.tray2 = False
				return "move tray 2 to assembly"
		if self.assembly == True and self.assembly_move == True:
			if self.tray1 == False:
				self.assembly = False
				self.assembly_move = False
				return "move from assembly to tray 1"
			elif self.tray2 == False:
				self.assembly = False
				self.assembly_move = False
				return "move from assembly to tray 2"

Tray_Pos = Tray_Pos()
Tray_Pos.get_position_from_bounding_box(0, 144, 335, 400, "partially full")
Tray_Pos.get_position_from_bounding_box(340, 288, 685, 575, "full")
print(Tray_Pos.check_move())
