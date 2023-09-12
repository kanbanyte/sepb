# Sample object detection programs

## YOLOv5 Sample

### Usage
Runs inference using a trained model and an image selected through a file dialog.
User can choose from the 2 options:
* **Version 0**
	- Designed to be used with tiling:
	- Model file selected via file dialog.
	- Allows the user to apply tiling with the specified row and column count.
* **Version 1**
	- Designed to be used with crop boxes:
	- Model file and crop box configured via configuration YAML file, which is selected by the user via a file dialog

### Notes
* If the file selection window does not appear, check if it opens in the background.
* The script is compatible with YOLOv5 models and requires a model file with a `.pt` extension.
* The tile dimensions must match the dimensions used to train the model. If no tiling was selected, enter (1,1)
* Scripts may output information to the console or save images to the file system.
* User prompts for file dialogs are provided in the file selection window name.

## Camera Inference Sample

### Usage
Runs inference using a trained model and an image selected through a file dialog. All configurations for the model and crop box are specified in the configuration YAML file, which is selected via file dialog

This sample is designed to be used with the ZED camera. The user can choose from 3 options:
* **Option 0**
	- Run chip detection model.
* **Option 1**
	- Run tray detection model.
* **Option 2**
	- Run case detection model.

### Notes
* If the file selection window does not appear, check if it opens in the background.
* The script is compatible with YOLOv5 models and requires a model file with a `.pt` extension.
* Scripts may output information to the console or save images to the file system.
* User prompts for file dialogs are provided in the file selection window name.
