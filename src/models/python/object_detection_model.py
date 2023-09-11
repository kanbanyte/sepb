from ultralytics import YOLO

class ObjectDetectionModel:
    def __init__(self, model_config):
        self.model = YOLO(model_config.get('file'))
        self.iou = model_config.get('iou')
        self.confidence = model_config.get('confidence')
        self.image_width = model_config.get('image_size').get('width')
        self.image_height = model_config.get('image_size').get('height')

    def run_inference(self, image):
        results = self.model.predict(image, verbose=False, conf=self.confidence, iou=self.iou, imgsz=(self.image_width, self.image_height))
        result = results[0]
        bounding_boxes = []
        if result.boxes is None or result.boxes.xyxy.numel() == 0:
            return []

        x1_tensor = result.boxes.xyxy[:, 0]
        y1_tensor = result.boxes.xyxy[:, 1]
        x2_tensor = result.boxes.xyxy[:, 2]
        y2_tensor = result.boxes.xyxy[:, 3]

        for (conf, x1, y1, x2, y2) in zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor):
            bounding_boxes.append((conf, x1, y1, x2, y2))
            # TODO: add code that logs results and save images with bounding boxes
            #  based on function arguments

        return bounding_boxes
