import unittest
from data_processing.case_position import convert_case_bounding_boxes
from models.detected_object import DetectedObject

class CasePositionTest(unittest.TestCase):

    def test_invalid_positions(self):
        invalid_bounding_boxes = [
            (7, 375, 101, 550),
            (7, 375, 101, 130),
            (7, 375, 101, 100),
            (7, 375, 101, 50),
            (7, 375, 101, -23),
            (7, 375, 101, 0),
            (7, 375, 101, 1000),
        ]

        for bounding_box in invalid_bounding_boxes:
            detected_object = DetectedObject(0.9, bounding_box)
            result = convert_case_bounding_boxes(detected_object)

            self.assertEqual(result, None, f"Position of case ({bounding_box}) should be {None}")

    def test_null_detection(self):
        with self.assertRaises(ValueError):
            convert_case_bounding_boxes(None)

    def test_all_positions(self):
        bounding_boxes = [
            (7, 375, 101, 499),  # position 1
            (7, 350, 101, 479), # position 2
            (8, 327, 101, 457), # position 3
            (8, 301, 101, 433), # position 4
            (8, 278, 101, 411), # position 5
            (7, 252, 99, 388), # position 6
            (7, 226, 100, 367), # position 7
            (7, 206, 99, 346), # position 8
            (7, 182, 99, 323), # position 9
            (8, 160, 99, 303), # position 10
            (7, 136, 99, 281), # position 11
            (8, 113, 99, 261), # position 12
            (8, 90, 98, 241), # position 13
            (4, 66, 97, 218), # position 14
            (8, 42, 99, 198), # position 15
            (9, 18, 97, 175), # position 16
            (10, 0, 98, 154) # position 17
        ]
        
        for i, bounding_box in enumerate(bounding_boxes):
            detected_object = DetectedObject(0.9, bounding_box)
            # position index starts at 1
            expected_position = i + 1
            result = convert_case_bounding_boxes(detected_object)
        
            self.assertEqual(result, expected_position, f"Position of case ({bounding_box}) should be {expected_position}")
    

if __name__ == '__main__':
    unittest.main()
