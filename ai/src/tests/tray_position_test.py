# FILEPATH: test_tray_position.py

import unittest
from data_processing.tray_position import determine_move
from models.detected_object import DetectedObject

#model classes:
#	0: empty
#	1: full
#	2: partially full
class model:
	classes = ["empty", "full", "partially full"]

class TestTrayPosition(unittest.TestCase):
	#tray assembly coords: (3, 157, 350, 418)
	#tray 1 coords: (352, 302, 690, 586)
	#tray 2 coords: (367, 7, 684, 259)

#region valid tests
	def test_tray1_assem_move(self):
		#tray 1 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, model).value, 3, "Should be move_tray1_assembly")

	def test_tray2_assem_move(self):
		#tray 2 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (367, 7, 684, 259))],
			0: [DetectedObject(0.9, (352, 302, 690, 586))]
		}
		self.assertEqual(determine_move(detections, model).value, 4, "Should be move_tray2_assembly")

	def test_assem_tray1_move(self):
		#assembly empty, tray 1 not present
		detections = {
			0: [DetectedObject(0.9,(3, 157, 350, 418))],
			2: [DetectedObject(0.9,(367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, model).value, 1, "Should be move_assembly_tray1")

	def test_assem_tray2_move(self):
		#assembly empty, tray 2 not present
		detections = {
			0: [DetectedObject(0.9, (3, 157, 350, 418))],
			1: [DetectedObject(0.9, (352, 302, 690, 586))]
		}
		self.assertEqual(determine_move(detections, model).value, 2, "Should be move_assembly_tray2")

	def test_no_move(self):
		#tray 1 empty, tray 2 empty
		detections = {
			0: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, model).value, 5, "Should be no_move")
#endregion

#region out of bounds tests
	def test_invalid_detection_bounding_boxes(self):
		invalid_detections = [
			{
				0: [DetectedObject(0.9, (352, 302, 690, 586))],
				0: [DetectedObject(0.9, (367, 7, 684, 1000))]
			},
			{
				0: [DetectedObject(0.9, (0,0,0,0))],
				0: [DetectedObject(0.9, (367, 7, 684, 259))]
			},
			{
				0: [DetectedObject(0.9, (-1,-1,-1,-1))],
				0: [DetectedObject(0.9, (367, 7, 684, 259))]
			}
		]

		for invalid_detection in invalid_detections:
			self.assertEqual(determine_move(invalid_detection, model).value, 5, "Should be no_move")

	def test_three_empty_trays(self):
		detections = {
			0: [DetectedObject(0.9, (3, 157, 350, 4181))],
			0: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, model).value, 5, "Should be no_move")
#endregion

if __name__ == '__main__':
	unittest.main()
