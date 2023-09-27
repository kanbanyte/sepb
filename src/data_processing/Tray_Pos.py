dict = {
	"assembly": False,
	"assembly_move": False,
	"tray1": False,
	"tray1_move": False,
	"tray2": False,
	"tray2_move": False
}

# checks if bounding box is between a certain range and returns a corresponding position
def get_position_from_bounding_box(bounding_box, tray_class):
	x1, y1, x2, y2 = bounding_box
	if x1 >=0  and y1 >= 140 or x2 <= 335 or y2 <= 400:
		dict.update({"assembly": True})
		if tray_class.lower() == "empty":
			dict.update({"assembly_move": True})
		return None
	if x1 >= 340 and y1 >= 0 or x2 <= 672 or y2 <= 265:
		dict.update({"tray2": True})
		if tray_class.lower() == "full":
			dict.update({"tray2_move": True})
		return None
	if x1 >= 340 and y1 >= 288 or x2 <= 685 or y2 <= 575:
		dict.update({"tray1": True})
		if tray_class.lower() == "full":
			dict.update({"tray1_move": True})
		return None

# checks if the trays are in a moveable state and returns a corresponding move
def check_move():
	if dict.get("assembly") == False:
		if dict.get("tray1_move") == True:
			dict.update({"tray1": False})
			dict.update({"tray1_move": False})
			return "move tray 1 to assembly"
		elif dict.get("tray2_move") == True:
			dict.update({"tray2_move": False})
			dict.update({"tray2": False})
			return "move tray 2 to assembly"
	if dict.get("assembly") == True and dict.get("assembly_move") == True:
		if dict.get("tray1") == False:
			dict.update({"assembly": False})
			dict.update({"assembly_move": False})
			return "move from assembly to tray 1"
		elif dict.get("tray2") == False:
			dict.update({"assembly": False})
			dict.update({"assembly_move": False})
			return "move from assembly to tray 2"

# #endregion

# this is test code

# calls the functions from the class, creating example bounding boxes and tray classes
get_position_from_bounding_box([0, 144, 335, 400], "empty")
get_position_from_bounding_box([340, 288, 685, 575], "full")
# calls the check_move function, this returns the relevant move text
print(check_move())
