
<!-- TOC ignore:true -->
# Sample object detection programs
**Table of Contents**
<!-- TOC -->

* [basic_sample.py](#basic_samplepy)
* [camera_sample.py](#camera_samplepy)
* [sample_config.yaml](#sample_configyaml)

<!-- /TOC -->

## basic_sample.py
<!-- TOC ignore:true -->
### Purpose
Used to demonstrate usage of a YOLO model class and quickly test trained models without involving the camera.
This sample mostly uses APIs supported by Ultralytics.

<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
User can choose from the 2 options:
* **Option 0**:
	* Model file (.pt file, not to be confused with yaml configuration file) selected via file dialogue.
	* Cropping is not supported.
	* Tiling is supported. Enter (1,1) to run inference on the original image
* **Option 1**:
	* Designed to be used with a yaml configuration file selected by the user via a file dialogue.
	* Cropping, model files and settings can be set in the configuration file.

## camera_sample.py
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

## sample_config.yaml
Sample configuration yaml file, containing all configurable properties with example values.
This file is intended as an example only, its values might not provide the best performance and should be tailored to your specific requirements.

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.
