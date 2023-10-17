# FILEPATH: test_tray_position.py

import unittest
from data_processing.tray_position import determine_move
from models.detected_object import DetectedObject
from data_processing.tray_position import TrayMovement

# model classes:
#	0: empty
#	1: full
#	2: partially full
class MockModel:
	classes = ["empty", "full", "partially full"]

class TestTrayPosition(unittest.TestCase):
	# tray assembly coords: (3, 157, 350, 418)
	# tray 1 coords: (352, 302, 690, 586)
	# tray 2 coords: (367, 7, 684, 259)

#region valid tests
	def test_tray1_to_assembly_move(self):
		#tray 1 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 3, f"Should be {TrayMovement.TRAY1_TO_ASSEMBLY.name}")

	def test_tray2_to_assembly_move(self):
		#tray 2 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (367, 7, 684, 259))],
			0: [DetectedObject(0.9, (352, 302, 690, 586))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 4, f"Should be {TrayMovement.TRAY2_TO_ASSEMBLY.name}")

	def test_assembly_to_tray1_move(self):
		#assembly empty, tray 1 not present
		detections = {
			0: [DetectedObject(0.9,(3, 157, 350, 418))],
			2: [DetectedObject(0.9,(367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 1, f"Should be {TrayMovement.ASSEMBLY_TO_TRAY1.name}")

	def test_assembly_to_tray2_move(self):
		#assembly empty, tray 2 not present
		detections = {
			0: [DetectedObject(0.9, (3, 157, 350, 418))],
			1: [DetectedObject(0.9, (352, 302, 690, 586))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 2, f"Should be {TrayMovement.ASSEMBLY_TO_TRAY2.name}")

	def test_NONE(self):
		#tray 1 empty, tray 2 empty
		detections = {
			0: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 5, f"Should be {TrayMovement.NONE.name}")
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
			self.assertEqual(determine_move(invalid_detection, MockModel).value, 5, f"Should be {TrayMovement.NONE.name}")

	def test_three_empty_trays(self):
		detections = {
			0: [DetectedObject(0.9, (3, 157, 350, 4181))],
			0: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}
		self.assertEqual(determine_move(detections, MockModel).value, 5, f"Should be {TrayMovement.NONE.name}")
#endregion

if __name__ == '__main__':
	unittest.main()
