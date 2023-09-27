# this is the class based code, this is likely the base way to do this, however it does introduce classes which can be finicky
# region class code
# class Tray_Pos:
# 	#defines global variables, these use the self tags so they can be accessed and then overwritten
# 	# in this, False means that the tray is not in that position, True means that it is
# 	# tray1_move and tray2_move are used to check if the tray is able to be moved,
# 	# this is used to check if the trays are in a moveable state for example entirely full for tray1 & 2 and empty for assembly
# 	def __init__(self):
# 		self.tray1 = False
# 		self.tray1_move = False
# 		self.tray2 = False
# 		self.tray2_move = False
# 		self.assembly = False
# 		self.assembly_move = False

# 	# checks if bounding box is between a certain range and returns a corresponding position
# 	def get_position_from_bounding_box(self, x1, y1, x2, y2, tray_class):
# 		if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
# 			self.assembly = True
# 			if tray_class == "empty":
# 				self.assembly_move = True
# 			return None
# 		if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
# 			self.tray2 = True
# 			if tray_class == "full":
# 				self.tray2_move = True
# 			return None
# 		if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
# 			self.tray1 = True
# 			if tray_class == "full":
# 				self.tray1_move = True
# 			return None

# 	def check_move(self):
# 		if self.assembly == False:
# 			if self.tray1_move == True:
# 				self.tray1 = False
# 				self.tray1_move = False
# 				return "move tray 1 to assembly"
# 			elif self.tray2_move == True:
# 				self.tray2_move = False
# 				self.tray2 = False
# 				return "move tray 2 to assembly"
# 		if self.assembly == True and self.assembly_move == True:
# 			if self.tray1 == False:
# 				self.assembly = False
# 				self.assembly_move = False
# 				return "move from assembly to tray 1"
# 			elif self.tray2 == False:
# 				self.assembly = False
# 				self.assembly_move = False
# 				return "move from assembly to tray 2"
#endregion

# for non class code refer here, this is bad practice but it works
# why is it bad practice? the global tag, generally you don't want to be modifying these but in this case it was difficult to get it working with it

# # region non class code

# 	#defines global variables, these use the self tags so they can be accessed and then overwritten
# 	# in this, False means that the tray is not in that position, True means that it is
# 	# tray1_move and tray2_move are used to check if the tray is able to be moved,
# 	# this is used to check if the trays are in a moveable state for example entirely full for tray1 & 2 and empty for assembly
tray1 = False
tray1_move = False
tray2 = False
tray2_move = False
assembly = False
assembly_move = False

# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(x1, y1, x2, y2, tray_class):
	global tray1,tray1_move, tray2, tray2_move, assembly, assembly_move
	if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
		assembly = True
		if tray_class == "Empty":
			assembly_move = True
		return None
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		tray2 = True
		if tray_class == "Full":
			tray2_move = True
		return None
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		tray1 = True
		if tray_class == "Full":
			tray1_move = True
		return None

# checks if the trays are in a moveable state and returns a corresponding move
def check_move():
	global tray1,tray1_move, tray2, tray2_move, assembly, assembly_move
	if assembly == False:
		if tray1_move == True:
			tray1 = False
			tray1_move = False
			return "move tray 1 to assembly"
		elif tray2_move == True:
			tray2_move = False
			tray2 = False
			return "move tray 2 to assembly"
	if assembly == True and assembly_move == True:
		if tray1 == False:
			assembly = False
			assembly_move = False
			return "move from assembly to tray 1"
		elif tray2 == False:
			assembly = False
			assembly_move = False
			return "move from assembly to tray 2"

# #endregion

# this is test code
'''
# calls the functions from the class, creating example bounding boxes and tray classes
get_position_from_bounding_box(0, 144, 335, 400, "empty")
get_position_from_bounding_box(340, 288, 685, 575, "full")
# calls the check_move function, this returns the relevant move text
print(check_move())
'''
