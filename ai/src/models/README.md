
<!-- TOC ignore:true -->
# Model APIs
**Table of Contents**
<!-- TOC -->

* [detected_object.py](#detected_objectpy)
* [object_detection_model.py](#object_detection_modelpy)

<!-- /TOC -->

## detected_object.py
Dataclass that holds the class index, confidence and crop box of detected objects.
The class name can be retrieved using the index and `ObjectDetectionModel.classes`.

## object_detection_model.py
Wrapper around the YOLO class from Ultralytics, created to narrow the functionalities and interface to fit this project.\
Requires a YAML configuration file to initialise the model and its parameters.
This class supports an option to save the image output and/or display it in a window.
To find more details about the configuration YAML file, see [samples/README.md](../samples/README.md)
