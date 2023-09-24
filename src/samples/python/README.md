
<!-- TOC ignore:true -->
# Sample object detection programs
**Table of Contents**
<!-- TOC -->

* [yolov5_sample.py](#yolov5_samplepy)
* [camera_inference_sample.py](#camera_inference_samplepy)

<!-- /TOC -->

## yolov5_sample.py
<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
User can optionally save the output image to a specified folder.\
Supports image tiling.\
To run the model on the full image, enter (1,1) for the tile dimensions.\
User can choose from the 2 options:
* **Option 0**:
	* Model file selected via file dialogue.
	* Cropping is not supported.
	* Use built-in methods from Ultralytics.
* **Option 1**:
	* Designed to be used with a yaml configuration file selected by the user via a file dialogue.
	* Cropping, model files and settings can be set in the configuration file.

## camera_inference_sample.py
<!-- TOC ignore:true -->
### Usage
Runs inference using a trained model and an image selected through a file dialogue.\
All configurations for the model and crop box are specified in the configuration YAML file, which is selected via file dialogue.

This sample is designed to be used with the ZED camera.\
The user can choose from 3 options:
* **Option 0**: Run chip detection model.
* **Option 1**: Run tray detection model.
* **Option 2**: Run case detection model.

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.
