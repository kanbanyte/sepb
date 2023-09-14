import os, sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../data_processing"))

from ultralytics import YOLO
from image_processing import draw_bounding_box
import cv2

class ObjectDetectionModel:
    def __init__(self, model_config):
        self.__model = YOLO(model_config.get('file'))
        self.__iou = model_config.get('iou')
        self.__confidence = model_config.get('confidence')
        self.__image_width = model_config.get('image_size').get('width')
        self.__image_height = model_config.get('image_size').get('height')

    def run_inference(self, image, result_img_path=None):
        """
        Draw a green bounding box on an image.

        Args:
            image (np.array): Input image as a NumPy array.
            result_img_path (None|str): optional file path to which the resulting image is saved.

        Returns:
            List of bounding boxes in the (left, top, right, bottom) format.
        """
        
        assert image.size != 0, f"Input image size must not be 0"
        
        # YOLO model class uses the (h,w) order
        image_dimension = (self.__image_height, self.__image_width)
        
        results = self.__model.predict(
            image, 
            verbose=False,
            conf=self.__confidence, 
            iou=self.__iou, 
            imgsz=image_dimension)
        
        result = results[0]
        if result.boxes is None or result.boxes.xyxy.numel() == 0:
            if result_img_path:
                # if the model detects no objects, we save the raw image so the user can still check the model's accuracy
                cv2.imwrite(result_img_path, image)
            return []

        x1_tensor = result.boxes.xyxy[:, 0]
        y1_tensor = result.boxes.xyxy[:, 1]
        x2_tensor = result.boxes.xyxy[:, 2]
        y2_tensor = result.boxes.xyxy[:, 3]

        bounding_boxes = []
        image_copy = image.copy()
        for (conf, x1, y1, x2, y2) in zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor):
            # the coordinates are still 1D tensors here and needs to be converted to a scalar value
            x1_int = int(x1)
            y1_int = int(y1)
            x2_int = int(x2)
            y2_int = int(y2)
            
            bounding_boxes.append((conf, x1_int, y1_int, x2_int, y2_int))
            if result_img_path:
                draw_bounding_box(image_copy, (x1_int, y1_int, x2_int, y2_int))
                cv2.imwrite(result_img_path, image_copy)

        return bounding_boxes
