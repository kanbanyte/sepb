__BOTTOM_Y_COORDINATE = 499
__TOP_Y_COORDINATE = 148
__POSITION_COUNT = 17
__POSITION_GAP = (__BOTTOM_Y_COORDINATE - __TOP_Y_COORDINATE) / (__POSITION_COUNT - 1)

def convert_case_bounding_boxes(detected_case):
	"""
	Convert the bounding boxes of cases into its position.
	The position values range from 1 to 17, with 1 being the position closest to the human operator.

	The x coordinates are ignored since the tight crop box should ensure that they are always valid.
	It is crucial that the crop box for both lens of the camera have the same y coordinates since they are used to compute the position.

	This function requires the case image to be cropped such that the image bottom aligns with the bottom of the horizontal T-slot bar and
	the height of the image is around 514px.

	Args:
		list(detected_object): a list of detected cases.

	Returns:
		int|None: Case position in the rack which ranges from 1 to 17 or None if the input cannot be converted to a valid position.
	"""
	if not detected_case:
		raise ValueError("Detected case object is null")

	_, _, _, detected_case_bottom_y = detected_case.bounding_box
	# without subtracting the value from __POSITION_COUNT, the function would return a position in reverse order.
	# 1 is added to the value to make sure the "inverted" position is correct
	position = __POSITION_COUNT - round((detected_case_bottom_y - __TOP_Y_COORDINATE)/ __POSITION_GAP) + 1
	if position < 1 or position > __POSITION_COUNT:
		return None

	return position
