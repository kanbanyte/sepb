
<!-- TOC ignore:true -->
# Model APIs
**Table of Contents**
<!-- TOC -->

* [detected_object.py](#camera_capturepy)
* [object_detection_model.py](#object_detection_modelpy)

<!-- /TOC -->

## detected_object.py
Dataclass that holds the confidence and crop box of detected objects.

## object_detection_model.py
Wrapper around the YOLO class from Ultralytics, created to narrow the functionalities and interface to fit this project.\
Requires a YAML configuration file to initialize the model and its parameters.
This class supports an option to save the image output and/or display it in a window.
