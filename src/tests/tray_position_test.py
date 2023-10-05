# FILEPATH: test_tray_position.py

import unittest
from tray_position import determine_move

class TestTrayPosition(unittest.TestCase):
    # invalid test cases
	def test_invalid_bounding_boxes(self):
		invalid_bounding_boxes = [
		#generate some random bounding boxes
		(-1, 139, 336, 401), #test if x1 < 0
		(339, -1, 673, 266), #test if y1 < 140
		(339, 287, 686, 576), #test if y1 < 140
		(686, 293, 345, 564), #test if x1 > x2
		(0,0,0,0)
		]
		for bounding_box in invalid_bounding_boxes:
			self.assertEqual(determine_move(bounding_box), -1)

	#tray assembly coords: (3, 157, 350, 418)
	#tray 1 coords: (352, 302, 690, 586)
	#tray 2 coords: (367, 7, 684, 259)

	def test_positions(self):
		bounding_boxes = [
		(3, 157, 350, 418), #tray assembly
		(352, 302, 690, 586), #tray 1
		(367, 7, 684, 259) #tray 2
		]
	#empty dictonary
	__false_detections = defaultdict(list)

	for i, bounding_box in enumerate(bounding_boxes):
		__false_detections[i].append(DetectedObject(confidence=0.9711142182350159, bounding_box=bounding_box))

	result = determine_move(__false_detections, model)



if __name__ == '__main__':
    unittest.main()
