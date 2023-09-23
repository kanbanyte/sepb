
<!-- TOC ignore:true -->
# Model
**Table of Contents**
<!-- TOC -->

* [detected_object.py](#camera_capturepy)
* [object_detection_model.py](#object_detection_modelpy)

<!-- /TOC -->

## detected_object.py
* Purpose:\
Dataclass that holds the confidence and crop box of detected objects.

## object_detection_model.py
* Purpose:\
Wrapper around the YOLO class from Ultralytics, created to narrow the functionalities and interface to fit this project.
Requires a YAML configuration file to initilize the model and its parameters.
