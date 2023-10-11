from dataclasses import dataclass

@dataclass
class DetectedObject:
    """
    Represents a detected object.

    Attributes:
        confidence (float): Confidence score of the detection.
        bounding_box (tuple): Bounding box coordinates in the format left-top-right-bottom.
    """
    confidence: float
    bounding_box: (int, int, int, int)
