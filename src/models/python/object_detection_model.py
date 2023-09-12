import os, sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../data_processing"))

from ultralytics import YOLO
from image_processing import draw_bounding_box
import cv2

class ObjectDetectionModel:
    def __init__(self, model_config):
        self.model = YOLO(model_config.get('file'))
        self.iou = model_config.get('iou')
        self.confidence = model_config.get('confidence')
        self.image_width = model_config.get('image_size').get('width')
        self.image_height = model_config.get('image_size').get('height')

    def run_inference(self, image, result_img_path=None):
        """
        Draw a green bounding box on an image.

        Args:
            image (np.array): Input image as a NumPy array.
            result_img_path (None|str): optional file path to which the resulting image is saved.

        Returns:
            List of bounding boxes in the (left, top, right, bottom) format.
        """
        results = self.model.predict(image, verbose=False, conf=self.confidence, iou=self.iou, imgsz=(self.image_width, self.image_height))
        result = results[0]
        bounding_boxes = []
        if result.boxes is None or result.boxes.xyxy.numel() == 0:
            if result_img_path:
                cv2.imwrite(result_img_path, image)
            return []

        x1_tensor = result.boxes.xyxy[:, 0]
        y1_tensor = result.boxes.xyxy[:, 1]
        x2_tensor = result.boxes.xyxy[:, 2]
        y2_tensor = result.boxes.xyxy[:, 3]

        image_copy = image.copy()
        for (conf, x1, y1, x2, y2) in zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor):
            bounding_boxes.append((conf, x1, y1, x2, y2))
            if result_img_path:
                draw_bounding_box(image_copy, (x1, y1, x2, y2))
                cv2.imwrite(result_img_path, image_copy)

        return bounding_boxes
