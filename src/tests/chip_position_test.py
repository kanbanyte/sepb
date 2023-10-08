import unittest
from data_processing.chip_position import get_chip_slot_number


class TestChipPosition(unittest.TestCase):

	def test_all_positons(self):
		# Test all positions
		valid_pos = [
			[13, 7, 31, 41],  # 1
			[11, 41, 31, 74],  # 2
			[12, 72, 31, 112],  # 3
			[15, 120, 29, 156],  # 4
			[15, 157, 27, 191],  # 5
			[13, 195, 26, 229],  # 6
			[65, 9, 81, 42],  # 7
			[65, 47, 82, 81],  # 8
			[62, 80, 80, 117],  # 9
			[65, 122, 83, 155],  # 10
			[65, 159, 78, 192],  # 11
			[65, 195, 79, 230],  # 12
			[118, 4, 129, 44],  # 13
			[112, 42, 129, 74],  # 14
			[109, 76, 129, 112],  # 15
			[113, 121, 129, 153],  # 16
			[110, 160, 128, 196],  # 17
			[113, 187, 128, 221],  # 18
			[169, 10, 183, 49],  # 19
			[168, 47, 182, 79],  # 20
			[167, 84, 180, 117],  # 21
			[165, 121, 182, 156],  # 22
			[165, 158, 180, 191],  # 23
			[164, 201, 180, 235],  # 24
			[206, 10, 225, 48],  # 25
			[205, 47, 223, 77],  # 26
			[204, 84, 224, 117],  # 27
			[207, 121, 226, 156],  # 28
			[206, 158, 225, 191],  # 29
			[209, 201, 228, 235],  # 30
			[267, 3, 283, 45],  # 31
			[267, 46, 283, 81],  # 32
			[265, 83, 284, 118],  # 33
			[267, 125, 285, 161],  # 34
			[267, 161, 285, 198],  # 35
			[267, 183, 285, 226],  # 36
			[317, 2, 334, 43],  # 37
			[317, 44, 334, 81],  # 38
			[315, 86, 335, 122],  # 39
			[317, 119, 334, 155],  # 40
			[317, 156, 331, 193],  # 41
			[318, 181, 335, 227],  # 42
			[368, 8, 385, 49],  # 43
			[367, 45, 384, 80],  # 44
			[367, 83, 384, 117],  # 45
			[369, 123, 385, 159],  # 46
			[368, 153, 387, 193],  # 47
			[369, 189, 385, 226]  # 48
		]
		for i, bounding_box in enumerate(valid_pos):
			self.assertEqual(get_chip_slot_number(bounding_box), i + 1, f"Position of chip{bounding_box} should be {i + 1}")

	def test_invalid_positons(self):
		invalid_bounding_boxes = [
		[0, 0, 0, 0], #null
		[65, 9, 81, 593], #exceptionally large
		[65, 9, 81, -4], #negative
		[65, 9, 81, 7], # y2 < y1
		[65, 9, 64, 42], #x2 < x1
		[65, 9, 710, 42], #exceptionally large
		[-9, 9, 81, 42] #negative
		]
		for bounding_box in invalid_bounding_boxes:
			self.assertEqual(get_chip_slot_number(bounding_box), None, f"Position of chip{bounding_box} should be none")
if __name__ == '__main__':
    unittest.main()
