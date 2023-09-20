import os, sys

from detected_object import DetectedObject
from collections import defaultdict
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../../data_processing"))

from ultralytics import YOLO
from image_processing import draw_bounding_box
import cv2

class ObjectDetectionModel:
    def __init__(self, model_config):
        self.__model = YOLO(model_config.get('file'))
        self.__classes = self.__model.names.copy()
        self.__iou = model_config.get('iou')
        self.__confidence = model_config.get('confidence')
        # TODO: add to config file
        self.__max_det = model_config.get('max_determination', 300)

        image_size = model_config.get('image_size')
        self.__image_width = self.__calculate_next_multiple(32, image_size.get('width'))
        self.__image_height = self.__calculate_next_multiple(32, image_size.get('height'))

    def __calculate_next_multiple(self, factor, number):
        """
        Calculates the multiple of `factor` and is cloest to `number` in the positive direction.
        YOLO models requires the image length to be a multiple of `factor` so it
        automatically converts the image size to that value and produce a warning message.
        We calculate that value to prevent this situation from the start.

        Args:
            factor (integer): Fasctor value.
            number (integer): Input value.

        Returns:
            Integer: multiple of `factor` closest to `number`
        """
        remainder = number % factor
        if remainder > 0:
            current_factor = number // factor
            return factor * (current_factor + 1)

        return number

    @property
    def classes(self):
        """
        Gets a dictionary containing indices and names of classes detected by the model.

        Returns:
            dict: dictionary mapping class index to class name.
        """
        return self.__classes

    def run_inference(self, image, result_img_path=None):
        """
        Draw a green bounding box on an image.

        Args:
            image (np.array): Input image as a NumPy array.
            result_img_path (None|str): optional file path to which the resulting image is saved.

        Returns:
            List[float, int, int, int, int]: List of confidence and bounding boxes in the (left, top, right, bottom) format.
        """

        assert image.size != 0, f"Input image size must not be 0"

        # YOLO model class uses the (h,w) order
        image_dimension = (self.__image_height, self.__image_width)

        results = self.__model.predict(
            image,
            verbose=False,
            conf=self.__confidence,
            iou=self.__iou,
            imgsz=image_dimension,
            max_det=self.__max_det)

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

        # defaultdict supports specifying a default for missing values
        detected_objects = defaultdict(list)
        image_copy = image.copy()
        for (object_class, conf, x1, y1, x2, y2) in zip(result.boxes.cls, result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor):
            # these values are still 1D tensors and need to be converted to scalar values
            object_class_index = int(object_class)
            bounding_box = (int(x1), int(y1), int(x2), int(y2))
            detected_object = DetectedObject(
                bounding_box=bounding_box,
                confidence=conf)

            detected_objects[object_class_index].append(detected_object)
            if result_img_path:
                draw_bounding_box(image_copy, bounding_box)
                cv2.imwrite(result_img_path, image_copy)

        return detected_objects