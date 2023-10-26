import unittest
from data_processing.chip_position import get_chip_slot_number


class TestChipPosition(unittest.TestCase):
	def test_all_positions(self):
		# Test all positions
		valid_pos = [
			# 1
			[13, 7, 31, 41],

			# 2
			[11, 41, 31, 74],

			# 3
			[12, 72, 31, 112],

			# 4
			[15, 120, 29, 156],

			# 5
			[15, 157, 27, 191],

			# 6
			[13, 195, 26, 229],

			# 7
			[65, 9, 81, 42],

			# 8
			[65, 47, 82, 81],

			# 9
			[62, 80, 80, 117],

			# 10
			[65, 122, 83, 155],

			# 11
			[65, 159, 78, 192],

			# 12
			[65, 195, 79, 230],

			# 13
			[118, 4, 129, 44],

			# 14
			[112, 42, 129, 74],

			# 15
			[109, 76, 129, 112],

			# 16
			[113, 121, 129, 153],

			# 17
			[110, 160, 128, 196],

			# 18
			[113, 187, 128, 221],

			# 19
			[169, 10, 183, 49],

			# 20
			[168, 47, 182, 79],

			# 21
			[167, 84, 180, 117],

			# 22
			[165, 121, 182, 156],

			# 23
			[165, 158, 180, 191],

			# 24
			[164, 201, 180, 235],

			# 25
			[206, 10, 225, 48],

			# 26
			[205, 47, 223, 77],

			# 27
			[204, 84, 224, 117],

			# 28
			[207, 121, 226, 156],

			# 29
			[206, 158, 225, 191],

			# 30
			[209, 201, 228, 235],

			# 31
			[267, 3, 283, 45],

			# 32
			[267, 46, 283, 81],

			# 33
			[265, 83, 284, 118],

			# 34
			[267, 125, 285, 161],

			# 35
			[267, 161, 285, 198],

			# 36
			[267, 183, 285, 226],

			# 37
			[317, 2, 334, 43],

			# 38
			[317, 44, 334, 81],

			# 39
			[315, 86, 335, 122],

			# 40
			[317, 119, 334, 155],

			# 41
			[317, 156, 331, 193],

			# 42
			[318, 181, 335, 227],

			# 43
			[368, 8, 385, 49],

			# 44
			[367, 45, 384, 80],

			# 45
			[367, 83, 384, 117],

			# 46
			[369, 123, 385, 159],

			# 47
			[368, 153, 387, 193],

			# 48
			[369, 189, 385, 226],
		]

		for i, bounding_box in enumerate(valid_pos):
			self.assertEqual(get_chip_slot_number(bounding_box), i + 1, f"Position of chip{bounding_box} should be {i + 1}")

	def test_invalid_positions(self):
		invalid_bounding_boxes = [
			# null
			[0, 0, 0, 0],

			# exceptionally large
			[65, 9, 81, 593],

			# negative
			[65, 9, 81, -4],

			# y2 < y1
			[65, 9, 81, 7],

			# x2 < x1
			[65, 9, 64, 42],

			# exceptionally large
			[65, 9, 710, 42],

			# negative
			[-9, 9, 81, 42]
		]

		for bounding_box in invalid_bounding_boxes:
			self.assertEqual(get_chip_slot_number(bounding_box), None, f"Position of chip {bounding_box} should be None")

if __name__ == '__main__':
    unittest.main()
