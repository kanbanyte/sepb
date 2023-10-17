
<!-- TOC ignore:true -->
# Sample object detection programs
**Table of Contents**
<!-- TOC -->

* [yolov5_sample.py](#yolov5_samplepy)
* [camera_inference_sample.py](#camera_inference_samplepy)

<!-- /TOC -->

## yolov5_sample.py
<!-- TOC ignore:true -->
### Purpose
Used to demonstrate usage of a YOLO model class.
This sample mostly uses APIs supported by Ultralytics.

<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
User can choose from the 2 options:
* **Option 0**:
	* Model file selected via file dialogue.
	* Cropping is not supported.
	* Tiling is supported. Enter (1,1) to run inference on the original image
* **Option 1**:
	* Designed to be used with a yaml configuration file selected by the user via a file dialogue.
	* Cropping, model files and settings can be set in the configuration file.

## camera_inference_sample.py
<!-- TOC ignore:true -->
### Purpose:
Runs inference using a trained model and an image captured by the ZED camera.
All configurations for the model, crop box, and camera settings are specified in the YAML file, which is selected via file dialogue.

<!-- TOC ignore:true -->
### Usage
The user can choose from 3 options:
* **Option 0**: Run chip detection model.
* **Option 1**: Run tray detection model.
* **Option 2**: Run case detection model.

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.
