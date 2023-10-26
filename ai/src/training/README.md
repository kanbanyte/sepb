
<!-- TOC ignore:true -->
# Training
**Table of Contents**
<!-- TOC -->

* [object_detection_training.ipynb](#object_detection_trainingipynb)
* [plot_ultralytics_result.py](#plot_ultralytics_resultspy)
* [Trained Directory](#trained-directory)

<!-- /TOC -->

## object_detection_training.ipynb
### Purpose:
Notebook that downloads the dataset from Roboflow and train the YOLO model.

### Usage
The user is allowed to input various parameters such as epochs, image size, batch size, frozen layers, etc.
If the notebook is run on Google Collab, the result folder of every training session can be downloaded as a zip file.

## plot_ultralytics_results.py
### Purpose:
Training the YOLO model from Ultralytics will produce several images showing the model's metrics.\
However, YOLO does not support displaying overlays of different metrics, such as showing the training curve and validation curve in the same graph.\
This script is created to address this issue by plotting them based on the data retrieved from the `results.csv` file produced after every training session.

### Usage
The user can select a `results.csv` file generated after a YOLO model is trained by Ultralytics.
If the `object_detection_training.ipynb` is used to train the model, the `results.csv` file is included in the output.

## Trained Directory
### Purpose
This directory contains trained models in the form of PyTorch files.
Full path to these files must be included in the configuration YAML so the model can be found and used to run inference.

### Usage
This section contains details of how these models are trained.
For best results, the size of images used in training should not be significantly different from those used in inference.
In training, the rectangular mode should be turned ON.
This setting converts a rectangular image into a square image with the size being the longer dimension by filling out the space with a solid color.
In inference, rectangular mode is turned on by default.
See [GitHub comment 1 ](https://github.com/ultralytics/yolov5/issues/2009#issuecomment-766147324) and [GitHub comment 2](https://github.com/ultralytics/yolov5/issues/2009#issuecomment-765557040) for more details.

The following section includes the settings used to train these models and recommended settings when running inference with them.

**detect_chip.pt:**
* Purpose: detect blue chips in red slots and yellow chips in white slots
* Class(es): `chips`
* Training settings:
	* Epochs: 100
	* Image size (rectangular mode turned ON): 416px
	* Dataset:
		* Version: 12
		* Size: 1194 images
		* Preprocessing:
			* Auto-Orient: Applied
		* Augmentation:
			* Flip: Horizontal, Vertical
			* Rotation: Between -30° and +30°
			* Shear: ±15° Horizontal, ±15° Vertical
			* Saturation: Between -10% and +10%
			* Brightness: Between -5% and +5%
			* Mosaic: Applied
			* Bounding Box: Flip: Horizontal, Vertical
* Recommended inference settings:
	* Image size: around 384px (W) x 256px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5

**detect_case.pt:**
* Purpose: detect blue chips in red slots and yellow chips in white slots
* Class(es): `case`
* Training settings:
	* Epochs: 80
	* Image size (rectangular mode turned ON): 544px
	* Dataset:
		* Version: 1
		* Size: 915 images
		* Preprocessing:
			* Auto-orient: applied
		* Augmentation:
			* Saturation: Between -20% and +20%
			* Brightness: Between -10% and +10%
			* Exposure: Between -10% and +10%
* Recommended inference settings:
	* Image size: around 117px (W) x 514px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5

**detect_tray.pt:**
* Purpose: detect white trays
* Class(es): `Full`, `Empty`, `partially Full` (inconsistent casing caused by typo when labelling the data)
* Training settings:
	* Epochs: 130
	* Image size (rectangular mode turned ON): 704px
	* Dataset:
		* Version: 6
		* Size: 594 images
		* Preprocessing:
			* Auto-orient: applied
		* Augmentation:
			* Flip: Horizontal, Vertical
			* Hue: Between -74° and +74°
			* Saturation: Between -25% and +25%
			* Exposure: Between -25% and +25%
			* Bounding Box: 90° Rotate: Clockwise, Counter-Clockwise, Upside Down
			* Bounding Box: Brightness: Between -15% and +15%
* Recommended inference settings:
	* Image sizes: around 661px (W) x 568px (H)
	* IoU threshold: 0.7
	* Confidence threshold: 0.5
