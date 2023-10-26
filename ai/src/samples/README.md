
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

<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
User can choose from the 2 options:
* **Option 0**:
	* Model file (.pt file, not to be confused with yaml configuration file) selected via file dialogue.
	* Cropping is not supported.
* **Option 1**:
	* Designed to be used with a YAML configuration file selected by the user via a file dialogue.
	* Cropping, model files and camera settings can be set in the configuration file.

## camera_sample.py
<!-- TOC ignore:true -->
### Purpose
Runs inference using a trained model and an image captured by the ZED camera.
All configurations for the model, crop box, and camera settings are specified in the YAML file, which is selected via file dialogue.

The recommended usage is to test a newly trained model with this script and verify the results before using it on the cobot.
The sample also demonstrates the use of APIs offered by this package.

<!-- TOC ignore:true -->
### Usage
The user can choose from 3 options:
* **Option 0**: Run chip detection model.
* **Option 1**: Run tray detection model.
* **Option 2**: Run case detection model.

## sample_config.yaml
Sample configuration yaml file, containing all configurable properties with example values.
This file is intended as an example only, its values might not provide the best performance and should be tailored to your specific requirements.

Depending on your code, not all properties declared in this files are accessed.
For example, if you plan to run inference using saved images rather than the camera, the `camera` portion can be omitted.
However, this is not guaranteed to be error-free so all properties should be declared, even with dummy values.

To find more information about default values for the model configuration and camera settings from their authors, visit the following pages:
* [Ultralytics Inference Arguments](https://docs.ultralytics.com/modes/predict/#inference-arguments)
* [ZED SDK Python API Reference](https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1VIDEO__SETTINGS.html)

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.
