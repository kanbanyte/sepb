# FILEPATH: test_tray_position.py

import unittest
from data_processing.tray_position import determine_move
from models.detected_object import DetectedObject
from data_processing.tray_position import CobotMovement

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

	def test_tray1_to_assembly_move(self):
		# tray 1 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (352, 302, 690, 586))],
			0: [DetectedObject(0.9, (367, 7, 684, 259))]
		}

		self.assertEqual(
			determine_move(detections, MockModel).value,
			CobotMovement.TRAY1_TO_ASSEMBLY.value,
			f"Should be {CobotMovement.TRAY1_TO_ASSEMBLY.name}"
		)

	def test_tray2_to_assembly_move(self):
		# tray 2 full, assembly not present
		detections = {
			1: [DetectedObject(0.9, (367, 7, 684, 259))],
			0: [DetectedObject(0.9, (352, 302, 690, 586))]
		}

		self.assertEqual(
			determine_move(detections, MockModel).value,
			CobotMovement.TRAY2_TO_ASSEMBLY.value,
			f"Should be {CobotMovement.TRAY2_TO_ASSEMBLY.name}"
		)

	def test_assembly_to_tray1_move(self):
		# assembly empty, tray 1 not present
		detections = {
			0: [DetectedObject(0.9,(3, 157, 350, 418))],
			2: [DetectedObject(0.9,(367, 7, 684, 259))]
		}

		self.assertEqual(
			determine_move(detections, MockModel).value,
			CobotMovement.ASSEMBLY_TO_TRAY1.value,
			f"Should be {CobotMovement.ASSEMBLY_TO_TRAY1.name}"
		)

	def test_assembly_to_tray2_move(self):
		detections = {
			# empty assembly
			0: [DetectedObject(0.9, (3, 157, 350, 418))],

			# absent tray 2
			1: [DetectedObject(0.9, (352, 302, 690, 586))]
		}

		self.assertEqual(
			determine_move(detections, MockModel).value,
			CobotMovement.ASSEMBLY_TO_TRAY2.value,
			f"Should be {CobotMovement.ASSEMBLY_TO_TRAY2.name}"
		)

	def test_start_tray1_load(self):
		test_cases = [
			{
				0: [
					# empty tray 1
					DetectedObject(0.9, (352, 302, 690, 586)),

					# empty tray 2
					DetectedObject(0.9, (367, 7, 684, 259))
				]
			},
			{
				0: [
					# empty tray 1
					DetectedObject(0.9, (352, 302, 690, 586)),
				],
				2: [
					# partially loaded tray 2
					DetectedObject(0.9, (367, 7, 684, 259))
				]
			},
			{
				0: [
					# empty tray 1
					DetectedObject(0.9, (352, 302, 690, 586)),
				],
				1: [
					# fully loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181))
				]
			},
			{
				0: [
					# empty tray 1
					DetectedObject(0.9, (352, 302, 690, 586)),
				],
				2: [
					# partially loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181))
				]
			}
		]

		for detections in test_cases:
			self.assertEqual(
				determine_move(detections, MockModel).value,
				CobotMovement.START_TRAY1_LOAD.value,
				f"Should be {CobotMovement.START_TRAY1_LOAD.name}"
			)

	def test_start_tray2_load(self):
		test_cases = [
			{
				0: [
					# empty tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				],
				2: [
					# partially loaded tray 1
					DetectedObject(0.9, (352, 302, 690, 586))
				]
			},
			{
				0: [
					# empty tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				],
				1: [
					# fully loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181))
				]
			},
			{
				0: [
					# empty tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				],
				2: [
					# partially loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181))
				]
			}
		]

		for i, detections in enumerate(test_cases):
			self.assertEqual(
				determine_move(detections, MockModel).value,
				CobotMovement.START_TRAY2_LOAD.value,
				f"Test case {i}. Should be {CobotMovement.START_TRAY2_LOAD.name}"
			)

	def test_continue_tray1_load(self):
		test_cases = [
			{
				1: [
					# fully loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181)),

					# fully loaded tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				],
				2: [
					# partially loaded tray 1
					DetectedObject(0.9, (352, 302, 690, 586))
				]
			},
			{
				2: [
					# partially loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181)),

					# partially loaded tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),

					# partially loaded tray 1
					DetectedObject(0.9, (352, 302, 690, 586))
				]
			}
		]

		for i, detections in enumerate(test_cases):
			self.assertEqual(
				determine_move(detections, MockModel).value,
				CobotMovement.CONTINUE_TRAY1_LOAD.value,
				f"Test case {i}. Should be {CobotMovement.CONTINUE_TRAY1_LOAD.name}"
			)

	def test_continue_tray2_load(self):
		test_cases = [
			{
				1: [
					# fully loaded tray 1
					DetectedObject(0.9, (352, 302, 690, 586)),

					# fully loaded assembly
					DetectedObject(0.9, (3, 157, 350, 4181)),
				],
				2: [
					# partially loaded tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				]
			},
			{
				2: [
					# partially loaded tray 2
					DetectedObject(0.9, (367, 7, 684, 259)),
				]
			},
		]

		for i, detections in enumerate(test_cases):
			self.assertEqual(
				determine_move(detections, MockModel).value,
				CobotMovement.CONTINUE_TRAY2_LOAD.value,
				f"Test case {i}. Should be {CobotMovement.CONTINUE_TRAY2_LOAD.name}"
			)

	def test_three_empty_trays(self):
		detections = {
			0: [
				# assembly tray
				DetectedObject(0.9, (3, 157, 350, 4181)),

				# tray 1
				DetectedObject(0.9, (352, 302, 690, 586)),

				# tray 2
				DetectedObject(0.9, (367, 7, 684, 259))
			],
		}

		self.assertEqual(
			determine_move(detections, MockModel).value,
			CobotMovement.NONE.value,
			f"Should be {CobotMovement.NONE.name}"
		)

if __name__ == '__main__':
	unittest.main()
